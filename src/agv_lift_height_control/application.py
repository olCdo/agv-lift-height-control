"""命令模式编排和 20 ms SSH 前台安全主循环。"""

from __future__ import annotations

import json
import math
import signal
import sys
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from time import monotonic, sleep
from typing import Any

from .calibration import (
    CalibrationBundle,
    CalibrationStore,
    LiftCalibrationSession,
    LowerCalibrationSession,
    UpperLimitSurvey,
    analyze_lift_trials,
    analyze_lower_trials,
)
from .can_pump import CanPump
from .config import AppConfig, load_config
from .controller import ControllerState, HeightController
from .modbus_rtu import ModbusRtuHeightSource
from .operator_runtime import (
    EOF_EVENT,
    CsvEventLogger,
    DeadmanAuthorizer,
    PosixAnsiTerminal,
    RuntimeSnapshot,
    SensorWorker,
    ShutdownLatch,
    SingleInstanceLock,
    TerminalEvent,
    validate_foreground_terminal,
)
from .passive_can import PassiveCanObserver
from .runtime_storage import CalibrationDraftStore
from .types import HeightSample, PumpCommand, PumpFeedback


@dataclass(frozen=True)
class CommandDecision:
    command: PumpCommand
    done: bool = False
    fatal_reason: str | None = None


class ZeroCommandSource:
    """monitor/observe/zero 模式的全零命令源，并声明允许的死手方向。"""

    def __init__(self, *, allow_lift: bool = False, allow_lower: bool = False) -> None:
        self.allow_lift = allow_lift
        self.allow_lower = allow_lower
        self.controller = None

    def step(
        self,
        now: float,
        sample: HeightSample | None,
        feedback: PumpFeedback | None,
        lift_authorized: bool,
        lower_authorized: bool,
    ) -> CommandDecision:
        return CommandDecision(PumpCommand.safe_stop())


class MoveCommandSource:
    allow_lift = True
    allow_lower = False

    def __init__(self, controller: HeightController) -> None:
        self.controller = controller

    def step(self, now, sample, feedback, lift_authorized, lower_authorized) -> CommandDecision:
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        return CommandDecision(
            self.controller.step(
                now=now,
                sample=sample,
                feedback=feedback,
                lift_authorized=lift_authorized,
                lower_authorized=lower_authorized,
            )
        )


class ManualLowerCommandSource(MoveCommandSource):
    allow_lift = False
    allow_lower = True


class LiftCalibrationCommandSource:
    allow_lift = True
    allow_lower = False
    def __init__(
        self,
        session: LiftCalibrationSession,
        *,
        controller: HeightController | None = None,
    ) -> None:
        self.session = session
        self.controller = controller

    def step(self, now, sample, feedback, lift_authorized, lower_authorized) -> CommandDecision:
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        desired = self.session.step(
            now=now,
            sample=sample,
            feedback=feedback,
            lift_authorized=lift_authorized,
        )
        command = (
            self.controller.step_external(
                now=now,
                sample=sample,
                feedback=feedback,
                desired_command=desired,
                lift_authorized=lift_authorized,
                lower_authorized=lower_authorized,
            )
            if self.controller is not None
            else desired
        )
        return CommandDecision(
            command,
            done=self.session.done,
            fatal_reason=self.session.fault_reason if self.session.failed else None,
        )

    def close(self) -> None:
        if self.controller is not None:
            self.controller.exit_external_mode()


class LowerCalibrationCommandSource:
    allow_lift = False
    allow_lower = True
    def __init__(
        self,
        session: LowerCalibrationSession,
        *,
        controller: HeightController | None = None,
    ) -> None:
        self.session = session
        self.controller = controller

    def step(self, now, sample, feedback, lift_authorized, lower_authorized) -> CommandDecision:
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        desired = self.session.step(
            now=now,
            sample=sample,
            feedback=feedback,
            lower_authorized=lower_authorized,
        )
        command = (
            self.controller.step_external(
                now=now,
                sample=sample,
                feedback=feedback,
                desired_command=desired,
                lift_authorized=lift_authorized,
                lower_authorized=lower_authorized,
            )
            if self.controller is not None
            else desired
        )
        return CommandDecision(
            command,
            done=self.session.done,
            fatal_reason=self.session.fault_reason if self.session.failed else None,
        )

    def close(self) -> None:
        if self.controller is not None:
            self.controller.exit_external_mode()


class SurveyCommandSource:
    allow_lift = True
    allow_lower = False

    def __init__(self, controller: HeightController, survey: UpperLimitSurvey) -> None:
        self.controller = controller
        self.survey = survey

    def step(self, now, sample, feedback, lift_authorized, lower_authorized) -> CommandDecision:
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        desired = self.survey.step(
            now=now, sample=sample, lift_authorized=lift_authorized
        )
        actual = self.controller.step_external(
            now=now,
            sample=sample,
            feedback=feedback,
            desired_command=desired,
            lift_authorized=lift_authorized,
            lower_authorized=lower_authorized,
        )
        return CommandDecision(
            actual,
            done=self.survey.limit_reached,
            fatal_reason=self.survey.fault_reason if self.survey.failed else None,
        )

    def close(self) -> None:
        self.controller.exit_external_mode()


def install_shutdown_signals(
    latch: ShutdownLatch,
    *,
    registrar: Callable[[int, Any], Any] = signal.signal,
    signal_module: Any = signal,
) -> None:
    """注册轻量处理器；处理器只写 ShutdownLatch，不做终端或硬件 I/O。"""

    def handler(signum: int, _frame: object) -> None:
        latch.request(f"signal:{signum}")

    for name in ("SIGHUP", "SIGTERM", "SIGINT"):
        number = getattr(signal_module, name, None)
        if number is not None:
            registrar(number, handler)


class ForegroundRuntime:
    """从单周期快照生成唯一实际命令，并统一执行先归零后停机。"""

    def __init__(
        self,
        *,
        mode: str,
        terminal: Any,
        logger: Any,
        clock: Callable[[], float] = monotonic,
        sleeper: Callable[[float], None] = sleep,
        shutdown: ShutdownLatch | None = None,
        pump: Any | None = None,
        sensor_worker: Any | None = None,
        observer: Any | None = None,
        loop_period_s: float = 0.02,
        motion_start_delay_s: float = 0.0,
        signal_installer: Callable[[ShutdownLatch], None] = install_shutdown_signals,
    ) -> None:
        if type(loop_period_s) not in {int, float} or not 0 < loop_period_s <= 0.1:
            raise ValueError("主循环周期必须在 0..0.1 秒内")
        if (
            type(motion_start_delay_s) not in {int, float}
            or not math.isfinite(float(motion_start_delay_s))
            or motion_start_delay_s < 0
        ):
            raise ValueError("动作启动延迟必须是非负有限秒数")
        self.mode = mode
        self.terminal = terminal
        self.logger = logger
        self.clock = clock
        self.sleeper = sleeper
        self.shutdown = shutdown or ShutdownLatch()
        self.pump = pump
        self.sensor_worker = sensor_worker
        self.observer = observer
        self.loop_period_s = float(loop_period_s)
        self.motion_start_delay_s = float(motion_start_delay_s)
        self.signal_installer = signal_installer
        self.authorizer = DeadmanAuthorizer(clock=clock)
        self._last_snapshot = RuntimeSnapshot(mode)
        self._last_logged_fault: str | None = None
        self._command_source: Any | None = None

    def run(
        self,
        command_source: Any,
        *,
        duration_s: float | None = None,
        max_iterations: int | None = None,
    ) -> RuntimeSnapshot:
        started_at = self.clock()
        iterations = 0
        primary_error: BaseException | None = None
        self._command_source = command_source
        motion_allowed_at = started_at
        try:
            self.signal_installer(self.shutdown)
            self.terminal.open()
            if self.sensor_worker is not None:
                self.sensor_worker.start()
            if self.observer is not None:
                self.observer.start()
            if self.pump is not None:
                self.pump.update_command(PumpCommand.safe_stop())
                self.pump.start()
            # 运行时长从所有资源成功启动后计；zero-can 因而会保留完整 5 秒 NMT 零窗。
            started_at = self.clock()
            if self.pump is not None:
                # CanPump 的 NMT 启动窗内实际只发零；会话也不得误把这段零输出计为试验。
                motion_allowed_at = started_at + self.motion_start_delay_s
            self.logger.log("start", self._last_snapshot)

            while True:
                if self.shutdown.requested:
                    self.authorizer.revoke_all()
                    self._force_zero()
                    self._refresh_event_snapshot(command=PumpCommand.safe_stop())
                    event_name = (
                        "signal"
                        if (self.shutdown.reason or "").startswith("signal:")
                        else "shutdown"
                    )
                    self._log_without_masking(event_name, detail=self.shutdown.reason)
                    break
                if max_iterations is not None and iterations >= max_iterations:
                    break
                now = self.clock()
                if duration_s is not None and now - started_at >= duration_s:
                    self.shutdown.request("duration")
                    continue

                event = self.terminal.read_event()
                if event is not None:
                    self._handle_event(event, command_source)
                    if self.shutdown.requested:
                        continue

                self._raise_background_error()
                sample = (
                    self.sensor_worker.latest_sample
                    if self.sensor_worker is not None
                    else None
                )
                if self.observer is not None:
                    feedback = self.observer.poll()
                elif self.pump is not None:
                    feedback = self.pump.last_feedback
                else:
                    feedback = None

                if self.pump is not None and now < motion_allowed_at:
                    decision = CommandDecision(PumpCommand.safe_stop())
                else:
                    decision = command_source.step(
                        now,
                        sample,
                        feedback,
                        self.authorizer.lift_authorized,
                        self.authorizer.lower_authorized,
                    )
                if not isinstance(decision, CommandDecision):
                    raise TypeError("命令源必须返回 CommandDecision")
                if self.pump is not None:
                    self.pump.update_command(decision.command)
                    displayed_command = getattr(
                        self.pump, "last_sent_command", decision.command
                    )
                else:
                    displayed_command = decision.command
                self._last_snapshot = self._snapshot(
                    command_source, sample, feedback, displayed_command
                )
                self.logger.log("cycle", self._last_snapshot)
                if (
                    self._last_snapshot.controller_fault
                    and self._last_snapshot.controller_fault != self._last_logged_fault
                ):
                    self.logger.log(
                        "fault",
                        self._last_snapshot,
                        detail=self._last_snapshot.controller_fault,
                    )
                    self._last_logged_fault = self._last_snapshot.controller_fault
                self.terminal.render(self._last_snapshot)
                if decision.fatal_reason is not None:
                    raise RuntimeError(decision.fatal_reason)
                if decision.done:
                    self.shutdown.request("completed")
                    continue
                iterations += 1
                self.sleeper(self.loop_period_s)
        except KeyboardInterrupt:
            self.authorizer.revoke_all()
            self._force_zero()
            self._refresh_event_snapshot(command=PumpCommand.safe_stop())
            self.shutdown.request("ctrl-c")
            self._log_without_masking("ctrl_c", detail="Ctrl+C")
        except BaseException as exc:
            primary_error = exc
            self.authorizer.revoke_all()
            try:
                self._force_zero()
                self._refresh_event_snapshot(command=PumpCommand.safe_stop())
            except BaseException:
                # cleanup 会再次尝试归零；原始异常仍作为首要根因向上传播。
                pass
            self._log_without_masking("fault", detail=str(exc))
            self._log_without_masking("runtime_error", detail=str(exc))
        finally:
            cleanup_error = self._safe_cleanup()
            if primary_error is None and cleanup_error is not None:
                primary_error = cleanup_error
        if primary_error is not None:
            raise primary_error
        return self._last_snapshot

    def _handle_event(self, event: TerminalEvent, command_source: Any) -> None:
        if event.kind == EOF_EVENT.kind:
            self.authorizer.revoke_all()
            self._force_zero()
            self._refresh_event_snapshot(command=PumpCommand.safe_stop())
            self.logger.log("eof", self._last_snapshot, detail="stdin EOF")
            self.shutdown.request("eof")
            return
        if event.kind != "key" or event.key is None:
            raise RuntimeError("未知终端事件")
        key = event.key.lower()
        detail = "ignored"
        if key == "u" and bool(getattr(command_source, "allow_lift", False)):
            self.authorizer.renew_lift()
            detail = "lift renewed 700ms"
        elif key == "d" and bool(getattr(command_source, "allow_lower", False)):
            self.authorizer.renew_lower()
            detail = "lower renewed 150ms"
        elif key == "q":
            self.authorizer.revoke_all()
            self._force_zero()
            self.shutdown.request("operator_q")
            detail = "safe quit"
        elif key == "c":
            controller = getattr(command_source, "controller", None)
            if controller is not None:
                controller.clear_fault()
                detail = "fault clear requested"
        self._refresh_event_snapshot(
            command=PumpCommand.safe_stop() if key == "q" else None
        )
        self.logger.log(
            "operator_key", self._last_snapshot, operator_key=event.key, detail=detail
        )
        if detail.startswith("lift renewed") or detail.startswith("lower renewed"):
            self.logger.log(
                "authorization",
                self._last_snapshot,
                operator_key=event.key,
                detail=detail,
            )

    def _raise_background_error(self) -> None:
        if self.sensor_worker is not None and self.sensor_worker.error:
            raise RuntimeError(self.sensor_worker.error)
        if self.observer is not None and getattr(self.observer, "error", None):
            raise RuntimeError(self.observer.error)
        if self.pump is not None and getattr(self.pump, "thread_fault", None):
            raise RuntimeError(self.pump.thread_fault)

    def _snapshot(self, source, sample, feedback, command) -> RuntimeSnapshot:
        controller = getattr(source, "controller", None)
        target = getattr(controller, "target_mm", None)
        state = getattr(controller, "state", None)
        state_text = getattr(state, "value", str(state) if state is not None else None)
        height = sample.height_mm if sample is not None else None
        error = target - height if target is not None and height is not None else None
        return RuntimeSnapshot(
            mode=self.mode,
            sample=sample,
            feedback=feedback,
            target_mm=target,
            target_error_mm=error,
            controller_state=state_text,
            command=command,
            lift_authorized=self.authorizer.lift_authorized,
            lower_authorized=self.authorizer.lower_authorized,
            lift_remaining_ms=self.authorizer.lift_remaining_ms,
            lower_remaining_ms=self.authorizer.lower_remaining_ms,
            controller_fault=getattr(controller, "fault_reason", None),
        )

    def _force_zero(self) -> None:
        if self.pump is not None:
            self.pump.update_command(PumpCommand.safe_stop())

    def _refresh_event_snapshot(self, *, command: PumpCommand | None = None) -> None:
        """按键/EOF/信号行必须记录事件发生后的授权与实际安全命令。"""
        self._last_snapshot = replace(
            self._last_snapshot,
            command=self._last_snapshot.command if command is None else command,
            lift_authorized=self.authorizer.lift_authorized,
            lower_authorized=self.authorizer.lower_authorized,
            lift_remaining_ms=self.authorizer.lift_remaining_ms,
            lower_remaining_ms=self.authorizer.lower_remaining_ms,
        )

    def _log_without_masking(self, event: str, **kwargs: object) -> None:
        try:
            self.logger.log(event, self._last_snapshot, **kwargs)
        except Exception:
            pass

    def _safe_cleanup(self) -> BaseException | None:
        """先归零并停泵，再用最后成功发送值记录可审计的 exit 行。"""
        first: BaseException | None = None

        def capture(operation: Any) -> None:
            nonlocal first
            if not callable(operation):
                return
            try:
                operation()
            except BaseException as exc:
                if first is None:
                    first = exc

        self.authorizer.revoke_all()
        try:
            self._force_zero()
        except BaseException as exc:
            first = exc
        # 外部模式、采样与观察器先收口，pump.stop 最后同步尽力补发零帧。
        for operation in (
            getattr(self._command_source, "close", None),
            getattr(self.sensor_worker, "stop", None),
            getattr(self.observer, "close", None),
            getattr(self.pump, "stop", None),
        ):
            capture(operation)

        actual = (
            getattr(self.pump, "last_sent_command", PumpCommand.safe_stop())
            if self.pump is not None
            else PumpCommand.safe_stop()
        )
        self._refresh_event_snapshot(command=actual)
        capture(
            lambda: self.logger.log(
                "exit", self._last_snapshot, detail=self.shutdown.reason
            )
        )
        capture(getattr(self.terminal, "close", None))
        capture(getattr(self.logger, "close", None))
        return first


@dataclass
class ApplicationDependencies:
    clock: Callable[[], float]
    sleeper: Callable[[float], None]
    source_factory: Callable[[Any], Any]
    worker_factory: Callable[[Any, float], Any]
    pump_factory: Callable[[Any], Any]
    observer_factory: Callable[[Any], Any]
    terminal_factory: Callable[[], Any]
    logger_factory: Callable[[Path, str], Any]
    lock_factory: Callable[[Path], Any]
    foreground_validator: Callable[[], None]
    signal_installer: Callable[[ShutdownLatch], None]
    stdout: Any


def default_dependencies() -> ApplicationDependencies:
    return ApplicationDependencies(
        clock=monotonic,
        sleeper=sleep,
        source_factory=lambda config: ModbusRtuHeightSource(config),
        worker_factory=lambda source, period: SensorWorker(source, poll_period_s=period),
        pump_factory=lambda config: CanPump(config),
        observer_factory=lambda config: PassiveCanObserver(config),
        terminal_factory=PosixAnsiTerminal,
        logger_factory=lambda directory, mode: CsvEventLogger(directory, mode),
        lock_factory=lambda path: SingleInstanceLock(path),
        foreground_validator=validate_foreground_terminal,
        signal_installer=install_shutdown_signals,
        stdout=sys.stdout,
    )


def run_application(args: Any, *, dependencies: ApplicationDependencies | None = None) -> int:
    """按 CLI 模式构造且只构造所需硬件边界。"""
    deps = dependencies or default_dependencies()
    config = load_config(args.config)
    calibration_store = CalibrationStore(config.storage.state_dir / "calibration.json")
    draft_store = CalibrationDraftStore(config.storage.state_dir / "lift-calibration-draft.json")

    if args.command == "show-calibration":
        print(
            json.dumps(
                calibration_store.load().to_json_object(),
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            ),
            file=deps.stdout,
        )
        return 0

    # 先验证 SSH 前台边界，失败时不创建锁、日志或连接硬件。
    deps.foreground_validator()
    lock = deps.lock_factory(config.storage.state_dir / "agv-lift-height-control.lock")
    lock.acquire()
    try:
        return _run_mode(args, config, calibration_store, draft_store, deps)
    finally:
        lock.release()


def _run_mode(args, config, calibration_store, draft_store, deps) -> int:
    mode = args.command
    # 下降阶段必须先验证跨进程草稿，再创建日志、串口或 CAN；坏草稿绝不进入现场动作。
    lift_draft = draft_store.load_lift() if mode == "calibrate-lower" else None
    if mode in {"monitor", "observe-can", "zero-can"}:
        source = ZeroCommandSource()
    else:
        # 控制模式先验证草稿/最终标定及目标边界，失败时不构造硬件工厂。
        source = _build_control_source(args, config, calibration_store)
    terminal = deps.terminal_factory()
    logger = deps.logger_factory(config.storage.log_dir, mode)
    worker = None
    pump = None
    observer = None
    duration = getattr(args, "duration_s", None)

    if mode == "monitor":
        worker = deps.worker_factory(deps.source_factory(config.sensor), config.sensor.poll_period_s)
    elif mode == "observe-can":
        observer = deps.observer_factory(config.can)
    elif mode == "zero-can":
        pump = deps.pump_factory(config.can)
    else:
        worker = deps.worker_factory(deps.source_factory(config.sensor), config.sensor.poll_period_s)
        pump = deps.pump_factory(config.can)

    runtime = ForegroundRuntime(
        mode=mode,
        terminal=terminal,
        logger=logger,
        clock=deps.clock,
        sleeper=deps.sleeper,
        shutdown=ShutdownLatch(),
        pump=pump,
        sensor_worker=worker,
        observer=observer,
        loop_period_s=min(0.02, config.control.control_loop_timeout_s),
        motion_start_delay_s=config.can.startup_nmt_s if pump is not None else 0.0,
        signal_installer=deps.signal_installer,
    )
    runtime.run(source, duration_s=duration)

    if mode == "calibrate-lift":
        if not source.session.done:
            raise RuntimeError("起升标定未完成，未保存草稿")
        draft_store.save_lift(analyze_lift_trials(source.session.trials))
    elif mode == "calibrate-lower":
        if not source.session.done:
            raise RuntimeError("下降标定未完成，未生成最终标定")
        lower = analyze_lower_trials(source.session.trials).confirm_comfortable(
            args.comfortable_valve
        )
        assert lift_draft is not None
        calibration_store.save(CalibrationBundle.from_results(lift_draft, lower))
    elif mode == "survey-upper":
        suggestion = source.survey.suggested_soft_limit_mm
        print(f"建议软上限: {suggestion:g} mm", file=deps.stdout)
        if args.confirm_save:
            source.survey.confirm(source.controller.calibration, store=calibration_store)
    return 0


def _build_control_source(args, config: AppConfig, calibration_store: CalibrationStore):
    if args.command == "calibrate-lift":
        controller = _existing_calibration_controller(
            calibration_store, config, ControllerState.LIFT_CALIBRATION
        )
        return LiftCalibrationCommandSource(
            LiftCalibrationSession(
                direction_tolerance_mm=config.control.direction_tolerance_mm,
                sensor_timeout_s=config.control.sensor_timeout_s,
                feedback_timeout_s=config.can.feedback_timeout_s,
                absolute_max_height_mm=config.control.absolute_max_height_mm,
            ),
            controller=controller,
        )
    if args.command == "calibrate-lower":
        controller = _existing_calibration_controller(
            calibration_store, config, ControllerState.LOWER_CALIBRATION
        )
        return LowerCalibrationCommandSource(
            LowerCalibrationSession(
                direction_tolerance_mm=config.control.direction_tolerance_mm,
                sensor_timeout_s=config.control.sensor_timeout_s,
                feedback_timeout_s=config.can.feedback_timeout_s,
                absolute_max_height_mm=config.control.absolute_max_height_mm,
            ),
            controller=controller,
        )

    bundle = calibration_store.load()
    controller = HeightController(
        config.control, bundle, feedback_timeout_s=config.can.feedback_timeout_s
    )
    if args.command == "move":
        controller.set_target(
            args.target_mm, temporary_max_height_mm=args.temporary_max_mm
        )
        return MoveCommandSource(controller)
    if args.command == "manual-lower":
        controller.set_manual_lower(True)
        return ManualLowerCommandSource(controller)
    if args.command == "survey-upper":
        controller.enter_external_mode(ControllerState.SURVEY)
        return SurveyCommandSource(
            controller,
            UpperLimitSurvey(
                config.control,
                bundle,
                temporary_max_height_mm=args.temporary_max_mm,
            ),
        )
    raise RuntimeError(f"不支持的运行模式: {args.command}")


def _existing_calibration_controller(
    store: CalibrationStore,
    config: AppConfig,
    mode: ControllerState,
) -> HeightController | None:
    """已有最终包时启用二级控制器门禁；首次标定保持 bootstrap 直通边界。"""
    if not store.path.exists():
        return None
    controller = HeightController(
        config.control,
        store.load(),
        feedback_timeout_s=config.can.feedback_timeout_s,
    )
    controller.enter_external_mode(mode)
    return controller
