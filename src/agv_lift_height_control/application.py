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
    CalibrationError,
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
from .runtime_storage import (
    LowerCalibrationDraftStore,
    SurveyDraft,
    SurveyDraftStore,
    calibration_fingerprint,
)
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
        feedback_timeout_s: float = 0.15,
        control_loop_timeout_s: float = 0.1,
        sensor_timeout_s: float = 0.1,
        max_speed_mm_s: float = 1200.0,
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
        for name, value, maximum in (
            ("反馈超时", feedback_timeout_s, 0.15),
            ("控制循环超时", control_loop_timeout_s, 0.1),
            ("传感器超时", sensor_timeout_s, 0.1),
        ):
            if (
                type(value) not in {int, float}
                or not math.isfinite(float(value))
                or not 0 < value <= maximum
            ):
                raise ValueError(f"{name}必须是 0..{maximum} 秒内的有限正数")
        if (
            type(max_speed_mm_s) not in {int, float}
            or not math.isfinite(float(max_speed_mm_s))
            or max_speed_mm_s <= 0
        ):
            raise ValueError("最大高度速度必须是有限正数")
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
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.control_loop_timeout_s = float(control_loop_timeout_s)
        self.sensor_timeout_s = float(sensor_timeout_s)
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.signal_installer = signal_installer
        self.authorizer = DeadmanAuthorizer(clock=clock)
        self._last_snapshot = RuntimeSnapshot(mode)
        self._last_logged_fault: str | None = None
        self._command_source: Any | None = None
        self._last_motion_cycle_at: float | None = None
        self._last_guard_sample: HeightSample | None = None

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
        self._last_motion_cycle_at = None
        self._last_guard_sample = None
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
                    self._refresh_event_snapshot(
                        command=self._last_actual_command(),
                        desired_command=PumpCommand.safe_stop(),
                        zero_requested=True,
                    )
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
                if type(now) not in {int, float} or not math.isfinite(float(now)) or now < 0:
                    raise RuntimeError("主循环时钟无效")
                now = float(now)

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

                motion_active = self.pump is not None and now >= motion_allowed_at
                if motion_active:
                    self._guard_motion_cycle(now, sample, feedback)
                if duration_s is not None and now - started_at >= duration_s:
                    self.shutdown.request("duration")
                    continue

                if self.pump is not None and not motion_active:
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
                    command_source,
                    sample,
                    feedback,
                    displayed_command,
                    decision.command,
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
            self._refresh_event_snapshot(
                command=self._last_actual_command(),
                desired_command=PumpCommand.safe_stop(),
                zero_requested=True,
            )
            self.shutdown.request("ctrl-c")
            self._log_without_masking("ctrl_c", detail="Ctrl+C")
        except BaseException as exc:
            primary_error = exc
            self.authorizer.revoke_all()
            try:
                self._force_zero()
                self._refresh_event_snapshot(
                    command=self._last_actual_command(),
                    desired_command=PumpCommand.safe_stop(),
                    zero_requested=True,
                )
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
            self._refresh_event_snapshot(
                command=self._last_actual_command(),
                desired_command=PumpCommand.safe_stop(),
                zero_requested=True,
            )
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
            command=self._last_actual_command(),
            desired_command=PumpCommand.safe_stop() if key == "q" else None,
            zero_requested=True if key == "q" else None,
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

    def _guard_motion_cycle(
        self,
        now: float,
        sample: HeightSample | None,
        feedback: PumpFeedback | None,
    ) -> None:
        """覆盖首次标定与控制器模式的统一 deadline、反馈和高度速度门禁。"""
        if self._last_motion_cycle_at is not None:
            gap = now - self._last_motion_cycle_at
            if gap < 0:
                raise RuntimeError("控制循环时钟回退")
            if gap - self.control_loop_timeout_s > 1e-12:
                raise RuntimeError(
                    f"控制循环超时: {gap:.6f}s > {self.control_loop_timeout_s:.6f}s"
                )
        self._last_motion_cycle_at = now

        pump_reason = getattr(self.pump, "fault_reason", None)
        if feedback is None:
            raise RuntimeError(f"CAN 0x197 反馈缺失；泵状态: {pump_reason or '未知'}")
        if (
            type(feedback.timestamp) not in {int, float}
            or not math.isfinite(float(feedback.timestamp))
        ):
            raise RuntimeError(f"CAN 0x197 反馈时间戳无效；泵状态: {pump_reason or '未知'}")
        feedback_age = now - float(feedback.timestamp)
        if feedback_age < 0 or feedback_age - self.feedback_timeout_s > 1e-12:
            raise RuntimeError(
                f"CAN 0x197 反馈超时或来自未来: age={feedback_age:.6f}s；"
                f"泵状态: {pump_reason or '未知'}"
            )
        if feedback.fault_code != 0:
            raise RuntimeError(
                f"CAN 0x197 故障码 {feedback.fault_code}；泵状态: {pump_reason or '未知'}"
            )
        if self.sensor_worker is not None:
            self._guard_height_sample(now, sample)

    def _guard_height_sample(self, now: float, sample: HeightSample | None) -> None:
        if (
            not isinstance(sample, HeightSample)
            or not sample.valid
            or sample.height_mm is None
        ):
            detail = sample.error if isinstance(sample, HeightSample) else "样本缺失"
            raise RuntimeError(f"高度样本无效: {detail or '未知原因'}")
        if (
            type(sample.timestamp) not in {int, float}
            or not math.isfinite(float(sample.timestamp))
        ):
            raise RuntimeError("高度样本时间戳无效")
        timestamp = float(sample.timestamp)
        previous = self._last_guard_sample
        if previous is not None and timestamp < previous.timestamp:
            raise RuntimeError("高度样本时间戳回退")
        age = now - timestamp
        if age < 0 or age - self.sensor_timeout_s > 1e-12:
            raise RuntimeError(f"高度样本超时或来自未来: age={age:.6f}s")
        if type(sample.height_mm) not in {int, float} or not math.isfinite(float(sample.height_mm)):
            raise RuntimeError("高度样本值无效")
        if previous is not None:
            if timestamp == previous.timestamp:
                if sample.height_mm != previous.height_mm:
                    raise RuntimeError("重复高度时间戳对应了不同高度")
                return
            assert previous.height_mm is not None
            speed = abs(float(sample.height_mm) - float(previous.height_mm)) / (
                timestamp - previous.timestamp
            )
            if speed - self.max_speed_mm_s > 1e-12:
                raise RuntimeError(
                    f"高度变化速度 {speed:.3f} mm/s 超过上限 {self.max_speed_mm_s:g} mm/s"
                )
        self._last_guard_sample = sample

    def _snapshot(
        self,
        source,
        sample,
        feedback,
        command,
        desired_command,
    ) -> RuntimeSnapshot:
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
            desired_command=desired_command,
            zero_requested=False,
            lift_authorized=self.authorizer.lift_authorized,
            lower_authorized=self.authorizer.lower_authorized,
            lift_remaining_ms=self.authorizer.lift_remaining_ms,
            lower_remaining_ms=self.authorizer.lower_remaining_ms,
            controller_fault=getattr(controller, "fault_reason", None),
            pump_fault=getattr(self.pump, "fault_reason", None),
        )

    def _force_zero(self) -> None:
        if self.pump is not None:
            self.pump.update_command(PumpCommand.safe_stop())

    def _last_actual_command(self) -> PumpCommand:
        if self.pump is None:
            return self._last_snapshot.command
        return getattr(self.pump, "last_sent_command", self._last_snapshot.command)

    def _refresh_event_snapshot(
        self,
        *,
        command: PumpCommand | None = None,
        desired_command: PumpCommand | None = None,
        zero_requested: bool | None = None,
    ) -> None:
        """事件行分开记录最后实际帧、最新期望命令与归零请求。"""
        self._last_snapshot = replace(
            self._last_snapshot,
            command=self._last_snapshot.command if command is None else command,
            desired_command=(
                self._last_snapshot.desired_command
                if desired_command is None
                else desired_command
            ),
            zero_requested=(
                self._last_snapshot.zero_requested
                if zero_requested is None
                else zero_requested
            ),
            lift_authorized=self.authorizer.lift_authorized,
            lower_authorized=self.authorizer.lower_authorized,
            lift_remaining_ms=self.authorizer.lift_remaining_ms,
            lower_remaining_ms=self.authorizer.lower_remaining_ms,
            pump_fault=getattr(self.pump, "fault_reason", None),
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
        self._refresh_event_snapshot(
            command=actual,
            desired_command=PumpCommand.safe_stop(),
            zero_requested=True,
        )
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
    lower_draft_store = LowerCalibrationDraftStore(
        config.storage.state_dir / "lower-calibration-draft.json"
    )
    survey_draft_store = SurveyDraftStore(
        config.storage.state_dir / "upper-survey-draft.json"
    )

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

    if args.command in {"confirm-lower", "confirm-upper"}:
        # 纯状态文件确认命令禁止触发 foreground_validator、TTY、串口或 CAN 工厂。
        lock = deps.lock_factory(config.storage.state_dir / "agv-lift-height-control.lock")
        lock.acquire()
        try:
            return _run_confirmation(
                args,
                config,
                calibration_store,
                draft_store,
                lower_draft_store,
                survey_draft_store,
                deps.stdout,
            )
        finally:
            lock.release()

    # 先验证 SSH 前台边界，失败时不创建锁、日志或连接硬件。
    deps.foreground_validator()
    lock = deps.lock_factory(config.storage.state_dir / "agv-lift-height-control.lock")
    lock.acquire()
    try:
        return _run_mode(
            args,
            config,
            calibration_store,
            draft_store,
            lower_draft_store,
            survey_draft_store,
            deps,
        )
    finally:
        lock.release()


def _run_confirmation(
    args,
    config,
    calibration_store,
    lift_draft_store,
    lower_draft_store,
    survey_draft_store,
    stdout,
) -> int:
    if args.command == "confirm-lower":
        lift = lift_draft_store.load_lift()
        lower = lower_draft_store.load().confirm_comfortable(args.comfortable_valve)
        calibration_store.save(CalibrationBundle.from_results(lift, lower))
        print(f"已确认舒适下降阀值: 0x{args.comfortable_valve:02X}", file=stdout)
        return 0

    bundle = calibration_store.load()
    draft = survey_draft_store.load()
    if draft.calibration_fingerprint != calibration_fingerprint(bundle):
        raise CalibrationError("上限测量草稿与当前标定包不匹配，禁止确认")
    chosen = args.soft_limit_mm
    maximum = min(
        draft.suggested_soft_limit_mm,
        config.control.absolute_max_height_mm,
        2900.0,
    )
    if chosen > maximum:
        raise CalibrationError(f"确认软上限不得超过安全建议 {maximum:g} mm")
    calibration_store.save(replace(bundle, soft_upper_limit_mm=chosen))
    print(f"已确认软上限: {chosen:g} mm", file=stdout)
    return 0


def _run_mode(
    args,
    config,
    calibration_store,
    draft_store,
    lower_draft_store,
    survey_draft_store,
    deps,
) -> int:
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
        feedback_timeout_s=config.can.feedback_timeout_s,
        control_loop_timeout_s=config.control.control_loop_timeout_s,
        sensor_timeout_s=config.control.sensor_timeout_s,
        max_speed_mm_s=config.control.max_speed_mm_s,
        signal_installer=deps.signal_installer,
    )
    runtime.run(source, duration_s=duration)

    if mode == "calibrate-lift":
        if not source.session.done:
            raise RuntimeError("起升标定未完成，未保存草稿")
        draft_store.save_lift(analyze_lift_trials(source.session.trials))
    elif mode == "calibrate-lower":
        if not source.session.done:
            raise RuntimeError("下降标定未完成，未保存下降草稿")
        lower = analyze_lower_trials(source.session.trials)
        lower_draft_store.save(lower)
        candidates = sorted(
            trial.valve
            for trial in lower.trials
            if trial.success
            and trial.direction_consistent
            and trial.displacement_mm >= 1.0
        )
        print(
            "下降动作测量已保存；成功候选: "
            + ", ".join(f"0x{valve:02X}" for valve in candidates),
            file=deps.stdout,
        )
    elif mode == "survey-upper":
        suggestion = source.survey.suggested_soft_limit_mm
        print(f"建议软上限: {suggestion:g} mm", file=deps.stdout)
        survey_draft_store.save(
            SurveyDraft(
                highest_observed_mm=source.survey.highest_observed_mm,
                suggested_soft_limit_mm=suggestion,
                temporary_max_height_mm=source.survey.temporary_max_height_mm,
                calibration_fingerprint=calibration_fingerprint(
                    source.controller.calibration
                ),
            )
        )
    return 0


def _build_control_source(args, config: AppConfig, calibration_store: CalibrationStore):
    if args.command == "calibrate-lift":
        temporary = getattr(args, "temporary_max_mm", None)
        if temporary is None:
            raise CalibrationError("起升标定必须显式提供临时最大高度")
        controller = _existing_calibration_controller(
            calibration_store, config, ControllerState.LIFT_CALIBRATION
        )
        limits = [float(temporary), config.control.absolute_max_height_mm, 2900.0]
        if controller is not None and controller.calibration.soft_upper_limit_mm is not None:
            limits.append(controller.calibration.soft_upper_limit_mm)
        effective_upper_limit = min(limits)
        return LiftCalibrationCommandSource(
            LiftCalibrationSession(
                direction_tolerance_mm=config.control.direction_tolerance_mm,
                sensor_timeout_s=config.control.sensor_timeout_s,
                feedback_timeout_s=config.can.feedback_timeout_s,
                absolute_max_height_mm=effective_upper_limit,
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
