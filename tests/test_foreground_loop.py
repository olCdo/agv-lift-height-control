import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from agv_lift_height_control import (
    EmergencyStopLatch,
    HeightSample,
    LiftHeightControl,
    PrepareLowerState,
    PumpCommand,
    PumpFeedback,
)
from agv_lift_height_control.application import (
    CommandDecision,
    ForegroundRuntime,
    LiftCalibrationCommandSource,
    LowerCalibrationCommandSource,
    ManualLowerCommandSource,
    MoveCommandSource,
    SurveyCommandSource,
    ZeroCommandSource,
    install_shutdown_signals,
)
from agv_lift_height_control.operator_runtime import (
    EOF_EVENT,
    RuntimeSnapshot,
    ShutdownLatch,
    TerminalEvent,
)
from agv_lift_height_control.passive_can import PassiveCanObserver, _create_receive_bus


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Terminal:
    def __init__(self, events=(), render_error=None):
        self.events = iter(events)
        self.render_error = render_error
        self.opened = 0
        self.closed = 0
        self.render_calls = 0

    def open(self):
        self.opened += 1

    def read_event(self):
        return next(self.events, None)

    def render(self, _snapshot):
        self.render_calls += 1
        if self.render_error:
            raise self.render_error

    def close(self):
        self.closed += 1


class Logger:
    def __init__(self, fail_event=None):
        self.events = []
        self.fail_event = fail_event
        self.closed = 0

    def log(self, event, snapshot=None, **kwargs):
        self.events.append((event, kwargs))
        if event == self.fail_event:
            raise RuntimeError("日志盘满")

    def close(self):
        self.closed += 1


class Pump:
    def __init__(self):
        self.actions = []
        self.last_feedback = PumpFeedback(0.0, 0, 0, 0)
        self.last_sent_command = PumpCommand.safe_stop()
        self.thread_fault = None

    def update_command(self, command):
        self.actions.append(("update", command))

    def start(self):
        self.actions.append(("start", None))

    def stop(self):
        self.actions.append(("stop", None))


class Worker:
    def __init__(self, error=None):
        self.latest_sample = HeightSample(0.0, 1, 100.0, True, None)
        self.error = error
        self.actions = []

    def start(self):
        self.actions.append("start")

    def stop(self):
        self.actions.append("stop")


def runtime(*, terminal=None, logger=None, pump=None, worker=None, clock=None, latch=None):
    clock = clock or Clock()
    return ForegroundRuntime(
        mode="test",
        terminal=terminal or Terminal([TerminalEvent.keypress("q")]),
        logger=logger or Logger(),
        clock=clock,
        sleeper=clock.sleep,
        shutdown=latch or ShutdownLatch(),
        pump=pump,
        sensor_worker=worker,
        loop_period_s=0.02,
        signal_installer=lambda _latch: None,
    )


@pytest.mark.parametrize(
    "events",
    [
        [EOF_EVENT],
        [TerminalEvent.keypress("q")],
    ],
)
def test_eof_and_normal_quit_send_zero_before_pump_stop(events) -> None:
    pump = Pump()
    runner = runtime(terminal=Terminal(events), pump=pump)

    runner.run(ZeroCommandSource(), max_iterations=2)

    stop_index = pump.actions.index(("stop", None))
    assert pump.actions[stop_index - 1] == ("update", PumpCommand.safe_stop())


def test_ctrl_c_sends_zero_before_stop() -> None:
    class InterruptTerminal(Terminal):
        def read_event(self):
            raise KeyboardInterrupt

    pump = Pump()
    runner = runtime(terminal=InterruptTerminal(), pump=pump)

    runner.run(ZeroCommandSource(), max_iterations=1)

    assert pump.actions[-2:] == [("update", PumpCommand.safe_stop()), ("stop", None)]


@pytest.mark.parametrize(
    ("worker_error", "render_error", "log_error"),
    [
        ("传感器线程失败", None, None),
        (None, RuntimeError("终端失败"), None),
        (None, None, "cycle"),
    ],
)
def test_worker_tui_and_logger_errors_zero_before_stop(
    worker_error, render_error, log_error
) -> None:
    pump = Pump()
    runner = runtime(
        terminal=Terminal(render_error=render_error),
        logger=Logger(log_error),
        pump=pump,
        worker=Worker(worker_error),
    )

    with pytest.raises(RuntimeError):
        runner.run(ZeroCommandSource(), max_iterations=1)

    stop_index = pump.actions.index(("stop", None))
    assert pump.actions[stop_index - 1] == ("update", PumpCommand.safe_stop())
    if log_error is None:
        assert "fault" in [event for event, _kwargs in runner.logger.events]


@pytest.mark.parametrize(
    ("signal_name", "signal_number"),
    [("SIGHUP", 1), ("SIGTERM", 15), ("SIGINT", 2)],
)
def test_signal_handler_never_touches_locked_latch_and_loop_stops_safely(
    signal_name, signal_number
) -> None:
    handlers = {}
    latch = ShutdownLatch()
    pump = Pump()
    fake_signals = SimpleNamespace(
        SIGHUP=1,
        SIGTERM=15,
        SIGINT=2,
    )

    def install(signal_target):
        install_shutdown_signals(
            signal_target,
            registrar=lambda number, handler: handlers.setdefault(number, handler),
            signal_module=fake_signals,
        )
        latch._lock.acquire()
        handler_thread = threading.Thread(
            target=handlers[signal_number], args=(signal_number, None)
        )
        try:
            handler_thread.start()
            handler_thread.join(timeout=0.1)
            assert not handler_thread.is_alive(), f"{signal_name} 处理器触碰了普通闩锁"
        finally:
            latch._lock.release()
            handler_thread.join(timeout=1.0)

    runner = ForegroundRuntime(
        mode="test",
        terminal=Terminal(),
        logger=Logger(),
        clock=Clock(),
        sleeper=lambda _seconds: None,
        shutdown=latch,
        pump=pump,
        loop_period_s=0.02,
        signal_installer=install,
    )
    runner.authorizer.renew_lift()

    runner.run(ZeroCommandSource(), max_iterations=1)

    assert latch.requested
    assert latch.reason == f"signal:{signal_number}"
    assert not runner.authorizer.lift_authorized
    assert pump.actions[-2:] == [("update", PumpCommand.safe_stop()), ("stop", None)]


def test_key_events_are_logged_one_by_one_and_only_renew_allowed_direction() -> None:
    logger = Logger()
    clock = Clock()
    runner = ForegroundRuntime(
        mode="move",
        terminal=Terminal(
            [
                TerminalEvent.keypress("u"),
                TerminalEvent.keypress("d"),
                TerminalEvent.keypress("u"),
                TerminalEvent.keypress("q"),
            ]
        ),
        logger=logger,
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        loop_period_s=0.02,
        signal_installer=lambda _latch: None,
    )

    runner.run(ZeroCommandSource(allow_lift=True), max_iterations=5)

    key_events = [item for item in logger.events if item[0] == "operator_key"]
    assert [item[1]["operator_key"] for item in key_events] == ["u", "d", "u", "q"]
    authorization_events = [item for item in logger.events if item[0] == "authorization"]
    assert [item[1]["operator_key"] for item in authorization_events] == ["u", "u"]
    assert runner.authorizer.lower_until <= clock.now


def test_signal_and_controller_fault_are_logged_as_distinct_events() -> None:
    logger = Logger()
    latch = ShutdownLatch()
    latch.request("signal:15")
    runtime(logger=logger, latch=latch).run(ZeroCommandSource(), max_iterations=1)
    assert "signal" in [event for event, _kwargs in logger.events]

    class FaultController(Controller):
        fault_reason = "控制器故障"

        def __init__(self):
            super().__init__()
            self.fault_reason = "控制器故障"

    clock = Clock()
    logger = Logger()
    source = MoveCommandSource(
        LiftHeightControl(FaultController(), EmergencyStopLatch(clock=clock))
    )
    ForegroundRuntime(
        mode="move",
        terminal=Terminal([None, TerminalEvent.keypress("q")]),
        logger=logger,
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        loop_period_s=0.02,
        signal_installer=lambda _latch: None,
    ).run(source, max_iterations=2)
    assert "fault" in [event for event, _kwargs in logger.events]


def test_authorization_and_eof_event_snapshots_reflect_immediate_deadman_state() -> None:
    class SnapshotLogger(Logger):
        def __init__(self):
            super().__init__()
            self.snapshots = []

        def log(self, event, snapshot=None, **kwargs):
            super().log(event, snapshot, **kwargs)
            self.snapshots.append((event, snapshot))

    logger = SnapshotLogger()
    clock = Clock()
    runner = ForegroundRuntime(
        mode="move",
        terminal=Terminal([TerminalEvent.keypress("u"), EOF_EVENT]),
        logger=logger,
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        loop_period_s=0.02,
        signal_installer=lambda _latch: None,
    )

    runner.run(ZeroCommandSource(allow_lift=True), max_iterations=2)

    authorization = next(snapshot for event, snapshot in logger.snapshots if event == "authorization")
    eof = next(snapshot for event, snapshot in logger.snapshots if event == "eof")
    assert authorization.lift_authorized is True
    assert authorization.lift_remaining_ms > 0
    assert eof.lift_authorized is eof.lower_authorized is False
    assert eof.command == PumpCommand.safe_stop()
    assert eof.desired_command == PumpCommand.safe_stop()
    assert eof.zero_requested is True


class Controller:
    def __init__(self):
        self.calls = []
        self.exits = 0
        self.state = SimpleNamespace(value="idle")
        self.fault_reason = None
        self.target_mm = 200.0

    def step(self, **kwargs):
        self.calls.append(("step", kwargs))
        return PumpCommand(interlock=True, lift_pwm=60)

    def step_external(self, **kwargs):
        self.calls.append(("step_external", kwargs))
        return kwargs["desired_command"]

    def exit_external_mode(self):
        self.exits += 1


def test_move_and_manual_lower_sources_use_controller_step_chain() -> None:
    sample = HeightSample(0.0, 1, 100.0, True, None)
    feedback = PumpFeedback(0.0, 0, 0, 0)
    controller = Controller()

    move_source = MoveCommandSource(
        LiftHeightControl(controller, EmergencyStopLatch(clock=lambda: 0.0))
    )
    move = move_source.step(0.0, sample, feedback, False, False)
    manual = ManualLowerCommandSource(controller).step(0.02, sample, feedback, False, True)

    assert move_source.allow_lift is move_source.allow_lower is False
    assert move_source.controller is controller
    assert move.command.lift_pwm == 60
    assert manual.command.lift_pwm == 60
    assert [call[0] for call in controller.calls] == ["step", "step"]
    assert controller.calls[0][1]["lift_authorized"] is True
    assert controller.calls[0][1]["lower_authorized"] is True
    assert controller.calls[1][1]["lift_authorized"] is False
    assert controller.calls[1][1]["lower_authorized"] is True


def test_survey_source_routes_desired_through_step_external() -> None:
    sample = HeightSample(0.0, 1, 100.0, True, None)
    feedback = PumpFeedback(0.0, 0, 0, 0)
    controller = Controller()

    class Survey:
        failed = False
        fault_reason = None
        limit_reached = False

        def step(self, **_kwargs):
            return PumpCommand(interlock=True, lift_pwm=50)

    decision = SurveyCommandSource(controller, Survey()).step(
        0.0, sample, feedback, True, False
    )

    assert decision.command.lift_pwm == 50
    assert controller.calls[0][0] == "step_external"


@pytest.mark.parametrize(
    ("source_type", "session_command", "expected_direction"),
    [
        (LiftCalibrationCommandSource, PumpCommand(interlock=True, lift_pwm=40), "lift"),
        (
            LowerCalibrationCommandSource,
            PumpCommand(interlock=True, lower_valve=0x10),
            "lower",
        ),
    ],
)
def test_existing_bundle_calibration_routes_session_desired_through_controller(
    source_type, session_command, expected_direction
) -> None:
    controller = Controller()

    class Session:
        done = False
        failed = False
        fault_reason = None

        def step(self, **_kwargs):
            return session_command

    sample = HeightSample(0.0, 1, 100.0, True, None)
    feedback = PumpFeedback(0.0, 0, 0, 0)
    source = source_type(Session(), controller=controller)

    decision = source.step(0.0, sample, feedback, True, True)

    assert controller.calls[0][0] == "step_external"
    assert controller.calls[0][1]["desired_command"] == session_command
    assert (decision.command.lift_pwm > 0) == (expected_direction == "lift")


def test_runtime_exits_existing_bundle_external_mode_during_safe_cleanup() -> None:
    controller = Controller()

    class Session:
        done = False
        failed = False
        fault_reason = None

        def step(self, **_kwargs):
            return PumpCommand.safe_stop()

    source = LiftCalibrationCommandSource(Session(), controller=controller)
    runner = runtime(terminal=Terminal([TerminalEvent.keypress("q")]))

    runner.run(source, max_iterations=1)

    assert controller.exits == 1


def test_loop_uses_20ms_period_without_real_sleep() -> None:
    clock = Clock()
    runner = runtime(terminal=Terminal([None, None, TerminalEvent.keypress("q")]), clock=clock)

    runner.run(ZeroCommandSource(), max_iterations=3)

    assert clock.now == pytest.approx(0.04)


def test_foreground_runtime_controls_at_50hz_but_renders_about_5hz() -> None:
    clock = Clock()
    terminal = Terminal([None] * 51)

    class CountingSource(ZeroCommandSource):
        def __init__(self):
            super().__init__()
            self.step_calls = 0

        def step(self, *args):
            self.step_calls += 1
            return super().step(*args)

    source = CountingSource()
    runner = runtime(terminal=terminal, clock=clock)

    runner.run(source, max_iterations=51)

    assert source.step_calls == 51
    assert terminal.render_calls == 6


def test_startup_nmt_window_does_not_advance_motion_session() -> None:
    clock = Clock()
    pump = Pump()

    class CountingSource(ZeroCommandSource):
        def __init__(self):
            super().__init__(allow_lift=True)
            self.calls = []

        def step(self, now, *args):
            self.calls.append(now)
            return CommandDecision(PumpCommand(interlock=True, lift_pwm=40))

    source = CountingSource()
    runner = ForegroundRuntime(
        mode="calibrate-lift",
        terminal=Terminal([None, None, None, TerminalEvent.keypress("q")]),
        logger=Logger(),
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        pump=pump,
        loop_period_s=0.02,
        motion_start_delay_s=0.04,
        signal_installer=lambda _latch: None,
    )

    runner.run(source, max_iterations=4)

    assert source.calls == [pytest.approx(0.04)]
    updates = [command for action, command in pump.actions if action == "update"]
    assert updates[:3] == [PumpCommand.safe_stop()] * 3
    assert updates[3].lift_pwm == 40


def test_csv_and_tui_snapshot_use_can_pump_last_sent_command_not_desired() -> None:
    class SnapshotLogger(Logger):
        def __init__(self):
            super().__init__()
            self.cycles = []

        def log(self, event, snapshot=None, **kwargs):
            super().log(event, snapshot, **kwargs)
            if event == "cycle":
                self.cycles.append(snapshot)

    class NonzeroSource(ZeroCommandSource):
        def step(self, *_args):
            return CommandDecision(PumpCommand(interlock=True, lift_pwm=40))

    pump = Pump()
    logger = SnapshotLogger()
    runner = runtime(
        terminal=Terminal([None, TerminalEvent.keypress("q")]),
        logger=logger,
        pump=pump,
    )

    runner.run(NonzeroSource(), max_iterations=2)

    assert any(command.lift_pwm == 40 for action, command in pump.actions if action == "update")
    assert logger.cycles[0].command == PumpCommand.safe_stop()


def test_prepare_lower_status_source_populates_target_state_and_fault_snapshot() -> None:
    source = SimpleNamespace(
        controller=None,
        status=SimpleNamespace(
            target_mm=100.0,
            state=PrepareLowerState.SETTLE,
            fault_reason="预升测试故障",
        ),
    )
    foreground = runtime()
    foreground.mode = "prepare-lower"

    value = foreground._snapshot(
        source,
        HeightSample(1.0, 20, 20.0, True, None),
        PumpFeedback(1.0, 0, 0, 0),
        PumpCommand.safe_stop(),
        PumpCommand.safe_stop(),
    )

    assert value.target_mm == 100.0
    assert value.target_error_mm == 80.0
    assert value.controller_state == "停泵观察"
    assert value.controller_fault == "预升测试故障"


def test_snapshot_reads_shared_emergency_stop_without_disguising_it_as_fault() -> None:
    latch = EmergencyStopLatch(clock=lambda: 1.0)
    latch.trigger("现场按钮")
    latch.trigger("后续原因")
    source = SimpleNamespace(
        controller=SimpleNamespace(
            target_mm=None,
            state=SimpleNamespace(value="emergency_stop"),
            fault_reason=None,
        ),
        emergency_stop_latch=latch,
    )

    value = runtime()._snapshot(
        source,
        HeightSample(1.0, 20, 20.0, True, None),
        PumpFeedback(1.0, 0, 0, 0),
        PumpCommand.safe_stop(),
        PumpCommand.safe_stop(),
    )

    assert value.emergency_stop_active is True
    assert value.emergency_stop_reason == "现场按钮"
    assert value.controller_fault is None


def test_exit_event_is_written_after_pump_stop_with_last_actual_command() -> None:
    order = []

    class OrderedPump(Pump):
        def stop(self):
            super().stop()
            order.append("pump.stop")

    class SnapshotLogger(Logger):
        def __init__(self):
            super().__init__()
            self.exit_snapshot = None

        def log(self, event, snapshot=None, **kwargs):
            super().log(event, snapshot, **kwargs)
            if event == "exit":
                self.exit_snapshot = snapshot
                order.append("logger.exit")

    pump = OrderedPump()
    # 模拟补零帧全部失败：desired 已归零，但总线最后成功发送的仍是非零。
    pump.last_sent_command = PumpCommand(interlock=True, lift_pwm=50)
    logger = SnapshotLogger()

    runtime(
        terminal=Terminal([TerminalEvent.keypress("q")]),
        logger=logger,
        pump=pump,
    ).run(ZeroCommandSource(), max_iterations=1)

    assert logger.exit_snapshot.command.lift_pwm == 50
    assert order == ["pump.stop", "logger.exit"]


def test_exit_log_failure_propagates_only_after_pump_is_stopped() -> None:
    pump = Pump()
    runner = runtime(
        terminal=Terminal([TerminalEvent.keypress("q")]),
        logger=Logger("exit"),
        pump=pump,
    )

    with pytest.raises(RuntimeError, match="日志盘满"):
        runner.run(ZeroCommandSource(), max_iterations=1)

    assert ("stop", None) in pump.actions


def test_q_event_records_last_successful_actual_separately_from_requested_zero() -> None:
    class SnapshotLogger(Logger):
        def __init__(self):
            super().__init__()
            self.q_snapshot = None

        def log(self, event, snapshot=None, **kwargs):
            super().log(event, snapshot, **kwargs)
            if event == "operator_key" and kwargs.get("operator_key") == "q":
                self.q_snapshot = snapshot

    pump = Pump()
    pump.last_sent_command = PumpCommand(interlock=True, lift_pwm=50)
    logger = SnapshotLogger()

    runtime(
        terminal=Terminal([TerminalEvent.keypress("q")]),
        logger=logger,
        pump=pump,
    ).run(ZeroCommandSource(), max_iterations=1)

    assert logger.q_snapshot.command.lift_pwm == 50
    assert logger.q_snapshot.desired_command == PumpCommand.safe_stop()
    assert logger.q_snapshot.zero_requested is True


@pytest.mark.parametrize(
    ("feedback", "reason"),
    [
        (None, "反馈缺失"),
        (PumpFeedback(0.0, 0, 0, 0), "反馈超时"),
        (PumpFeedback(0.2, 0, 7, 0), "故障码"),
    ],
)
def test_motion_window_end_requires_fresh_fault_free_197_feedback(feedback, reason) -> None:
    clock = Clock()
    pump = Pump()
    pump.last_feedback = feedback
    clock.now = 0.2
    runner = ForegroundRuntime(
        mode="calibrate-lift",
        terminal=Terminal([None]),
        logger=Logger(),
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        pump=pump,
        loop_period_s=0.02,
        motion_start_delay_s=0.0,
        feedback_timeout_s=0.15,
        signal_installer=lambda _latch: None,
    )

    with pytest.raises(RuntimeError, match=reason):
        runner.run(ZeroCommandSource(allow_lift=True), max_iterations=1)

    assert pump.actions[-1] == ("stop", None)


def test_runtime_timestamps_cycle_after_reading_concurrent_can_snapshot() -> None:
    """CAN线程在快照读取时更新反馈，不得被旧的主循环时间误判为未来。"""
    clock = Clock()
    source_calls = []

    class ConcurrentFeedbackPump:
        def __init__(self):
            self.actions = []
            self.last_sent_command = PumpCommand.safe_stop()
            self.thread_fault = None
            self.fault_reason = None

        @property
        def last_feedback(self):
            clock.now += 0.00075
            return PumpFeedback(clock.now, 0, 0, 0)

        def update_command(self, command):
            self.actions.append(("update", command))

        def start(self):
            self.actions.append(("start", None))

        def stop(self):
            self.actions.append(("stop", None))

    class Source(ZeroCommandSource):
        def step(self, now, *_args):
            source_calls.append(now)
            return CommandDecision(PumpCommand.safe_stop())

    pump = ConcurrentFeedbackPump()
    runner = ForegroundRuntime(
        mode="calibrate-lift",
        terminal=Terminal([None]),
        logger=Logger(),
        clock=clock,
        sleeper=clock.sleep,
        shutdown=ShutdownLatch(),
        pump=pump,
        loop_period_s=0.02,
        motion_start_delay_s=0.0,
        signal_installer=lambda _latch: None,
    )

    runner.run(Source(), max_iterations=1)

    assert source_calls == [pytest.approx(0.00075)]
    assert pump.actions[-1] == ("stop", None)


def test_bootstrap_motion_loop_deadline_gap_locks_stop_before_next_session_step() -> None:
    clock = Clock()
    pump = Pump()
    source_calls = []

    class Source(ZeroCommandSource):
        def step(self, now, *_args):
            source_calls.append(now)
            return CommandDecision(PumpCommand.safe_stop())

    def slow_sleep(_seconds):
        clock.now += 0.101
        pump.last_feedback = PumpFeedback(clock.now, 0, 0, 0)

    runner = ForegroundRuntime(
        mode="calibrate-lift",
        terminal=Terminal([None, None]),
        logger=Logger(),
        clock=clock,
        sleeper=slow_sleep,
        shutdown=ShutdownLatch(),
        pump=pump,
        loop_period_s=0.02,
        control_loop_timeout_s=0.1,
        signal_installer=lambda _latch: None,
    )

    with pytest.raises(RuntimeError, match="控制循环"):
        runner.run(Source(), max_iterations=2)

    assert source_calls == [0.0]


@pytest.mark.parametrize(
    ("samples", "reason"),
    [
        (
            [
                HeightSample(0.0, 1, 100.0, True, None),
                HeightSample(0.02, 2, 130.0, True, None),
            ],
            "速度",
        ),
        (
            [
                HeightSample(0.0, 1, 100.0, True, None),
                HeightSample(0.0, 2, 101.0, True, None),
            ],
            "重复",
        ),
        (
            [
                HeightSample(0.0, 1, 100.0, True, None),
                HeightSample(-0.01, 2, 100.0, True, None),
            ],
            "回退",
        ),
        (
            [
                HeightSample(0.0, 1, 100.0, True, None),
                HeightSample(0.02, None, None, False, "串口失败"),
            ],
            "无效",
        ),
    ],
)
def test_shared_motion_height_guard_rejects_jump_duplicate_rollback_and_invalid(
    samples, reason
) -> None:
    clock = Clock()
    pump = Pump()

    class SequenceWorker(Worker):
        def __init__(self):
            super().__init__()
            self.index = 0

        @property
        def latest_sample(self):
            value = samples[min(self.index, len(samples) - 1)]
            self.index += 1
            return value

        @latest_sample.setter
        def latest_sample(self, _value):
            pass

    def advance(seconds):
        clock.now += seconds
        pump.last_feedback = PumpFeedback(clock.now, 0, 0, 0)

    runner = ForegroundRuntime(
        mode="calibrate-lift",
        terminal=Terminal([None, None]),
        logger=Logger(),
        clock=clock,
        sleeper=advance,
        shutdown=ShutdownLatch(),
        pump=pump,
        sensor_worker=SequenceWorker(),
        loop_period_s=0.02,
        sensor_timeout_s=0.1,
        max_speed_mm_s=1200.0,
        signal_installer=lambda _latch: None,
    )

    with pytest.raises(RuntimeError, match=reason):
        runner.run(ZeroCommandSource(), max_iterations=2)


class Frame:
    arbitration_id = 0x197
    is_extended_id = False
    is_remote_frame = False
    is_error_frame = False
    dlc = 8
    data = bytes((0, 0, 0, 1, 0, 0, 0, 2))


class ReceiveOnlyBus:
    def __init__(self):
        self.shutdowns = 0

    def recv(self, timeout=0):
        assert timeout == 0
        return Frame()

    def send(self, *_args, **_kwargs):
        raise AssertionError("observe-can 禁止发送")

    def shutdown(self):
        self.shutdowns += 1


def test_passive_can_observer_parses_197_without_any_send() -> None:
    bus = ReceiveOnlyBus()
    config = SimpleNamespace(interface="can0", bitrate=500000)
    observer = PassiveCanObserver(
        config,
        bus_factory=lambda _interface: bus,
        link_checker=lambda *_args: None,
        clock=lambda: 3.0,
    )
    observer.start()

    feedback = observer.poll()
    observer.close()

    assert feedback == PumpFeedback(3.0, 1, 0, 2)
    assert bus.shutdowns == 1


def test_passive_can_poll_returns_after_bounded_continuous_noise_then_reads_feedback() -> None:
    class NoiseThenFeedbackBus:
        def __init__(self):
            self.noise = True
            self.calls = 0

        def recv(self, timeout=0):
            assert timeout == 0
            self.calls += 1
            if self.noise:
                if self.calls > 256:
                    raise AssertionError("单次 poll 未在连续噪声中有界返回")
                return SimpleNamespace(arbitration_id=0x123)
            return Frame()

    bus = NoiseThenFeedbackBus()
    observer = PassiveCanObserver(
        SimpleNamespace(interface="can0", bitrate=500000),
        bus_factory=lambda _interface: bus,
        link_checker=lambda *_args: None,
        clock=lambda: 4.0,
    )
    observer.start()

    assert observer.poll() is None
    first_poll_calls = bus.calls
    bus.noise = False
    assert observer.poll() == PumpFeedback(4.0, 1, 0, 2)
    assert first_poll_calls <= 256


def install_lazy_can_interface(monkeypatch, create_bus):
    """安装与现场一致的延迟导出 python-can 测试包。"""
    can_package = ModuleType("can")
    can_package.__path__ = []  # type: ignore[attr-defined]
    interface_module = ModuleType("can.interface")
    interface_module.Bus = create_bus  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "can", can_package)
    monkeypatch.setitem(sys.modules, "can.interface", interface_module)
    return can_package


def test_default_receive_bus_requests_exact_standard_197_filter(monkeypatch) -> None:
    calls = []
    expected_bus = object()

    def create_bus(**kwargs):
        calls.append(kwargs)
        return expected_bus

    install_lazy_can_interface(monkeypatch, create_bus)

    assert _create_receive_bus("can0") is expected_bus
    assert calls == [
        {
            "interface": "socketcan",
            "channel": "can0",
            "can_filters": [{"can_id": 0x197, "can_mask": 0x7FF, "extended": False}],
        }
    ]


def test_default_receive_bus_imports_lazy_python_can_interface_submodule(monkeypatch) -> None:
    """复现现场包：顶层 ``can`` 不预先暴露 ``interface`` 属性。"""
    calls = []
    expected_bus = object()

    def create_bus(**kwargs):
        calls.append(kwargs)
        return expected_bus

    can_package = install_lazy_can_interface(monkeypatch, create_bus)

    assert not hasattr(can_package, "interface")
    assert _create_receive_bus("can0") is expected_bus
    assert calls[0]["channel"] == "can0"


def test_default_receive_bus_falls_back_when_python_can_rejects_filters(monkeypatch) -> None:
    calls = []
    expected_bus = object()

    def create_bus(**kwargs):
        calls.append(kwargs)
        if "can_filters" in kwargs:
            raise TypeError("旧版 python-can 不支持 can_filters")
        return expected_bus

    install_lazy_can_interface(monkeypatch, create_bus)

    assert _create_receive_bus("can0") is expected_bus
    assert "can_filters" in calls[0]
    assert "can_filters" not in calls[1]
