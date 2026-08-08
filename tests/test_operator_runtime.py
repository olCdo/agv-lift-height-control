import csv
import io
import json
import sys
import threading
from types import ModuleType

import pytest

import agv_lift_height_control as package
import agv_lift_height_control.operator_runtime as operator_runtime
from agv_lift_height_control import HeightSample, PumpCommand, PumpFeedback
from agv_lift_height_control.calibration import (
    LIFT_PWM_LEVELS,
    LOWER_VALVE_LEVELS,
    CalibrationBundle,
    CalibrationError,
    LiftCalibrationResult,
    LiftTrial,
    LowerCalibrationResult,
    LowerTrial,
)
from agv_lift_height_control.operator_runtime import (
    CSV_FIELDS,
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
from agv_lift_height_control.runtime_storage import (
    CalibrationDraftStore,
    LowerCalibrationDraftStore,
    SurveyDraft,
    SurveyDraftStore,
    calibration_fingerprint,
    lift_calibration_fingerprint,
)


def test_runtime_public_interfaces_are_exported_from_package() -> None:
    for name in (
        "StorageConfig",
        "DeadmanAuthorizer",
        "SensorWorker",
        "CsvEventLogger",
        "CalibrationDraftStore",
        "LowerCalibrationDraftStore",
        "SurveyDraft",
        "SurveyDraftStore",
        "ForegroundRuntime",
        "PassiveCanObserver",
    ):
        assert hasattr(package, name), name


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


class FakeStream:
    def __init__(self, tty=True):
        self.tty = tty

    def isatty(self):
        return self.tty


def test_deadman_authorizations_have_independent_exact_boundaries() -> None:
    clock = FakeClock(10.0)
    auth = DeadmanAuthorizer(clock=clock)

    auth.renew_lift()
    assert auth.lift_remaining_ms == 700
    clock.now = 10.69
    auth.renew_lower()
    assert auth.lower_remaining_ms == 150
    assert auth.lift_authorized is True
    assert auth.lower_authorized is True

    clock.now = 10.7
    assert auth.lift_authorized is False
    assert auth.lower_authorized is True
    clock.now = 10.84
    assert auth.lower_authorized is False


def test_deadman_direction_events_do_not_extend_other_direction() -> None:
    clock = FakeClock(1.0)
    auth = DeadmanAuthorizer(clock=clock)
    auth.renew_lift()
    clock.now = 1.69
    auth.renew_lower()
    clock.now = 1.7

    assert auth.lift_authorized is False
    assert auth.lower_authorized is True
    auth.revoke_all()
    assert auth.lift_remaining_ms == auth.lower_remaining_ms == 0


@pytest.mark.parametrize(
    ("stdin_tty", "stdout_tty", "env", "message"),
    [
        (False, True, {}, "stdin"),
        (True, False, {}, "stdout"),
        (True, True, {"TMUX": "/tmp/tmux"}, "tmux"),
        (True, True, {"STY": "1.screen"}, "screen"),
        (True, True, {"TERM": "dumb"}, "TERM=dumb"),
    ],
)
def test_foreground_terminal_rejects_unsupported_sessions(
    stdin_tty, stdout_tty, env, message
) -> None:
    with pytest.raises(RuntimeError, match=message):
        validate_foreground_terminal(
            stdin=FakeStream(stdin_tty), stdout=FakeStream(stdout_tty), environ=env
        )


def test_foreground_terminal_rejects_background_process_group_and_empty_tmux_marker() -> None:
    with pytest.raises(RuntimeError, match="后台进程组"):
        validate_foreground_terminal(
            stdin=FakeStream(),
            stdout=FakeStream(),
            environ={},
            foreground_checker=lambda _stdin: False,
        )
    with pytest.raises(RuntimeError, match="tmux"):
        validate_foreground_terminal(
            stdin=FakeStream(), stdout=FakeStream(), environ={"TMUX": ""}
        )


def test_terminal_events_preserve_each_character_and_eof() -> None:
    assert TerminalEvent.keypress("u") != TerminalEvent.keypress("u")
    assert EOF_EVENT.kind == "eof"


def test_tui_render_clears_every_line_before_writing_shorter_values() -> None:
    """每行都要先清除，否则 SSH 终端会把旧的第三位残留到新两位故障码后。"""
    output = io.StringIO()
    terminal = PosixAnsiTerminal(stdout=output)

    terminal.render(
        RuntimeSnapshot(
            mode="observe-can",
            feedback=PumpFeedback(1.0, -3, 0x52, 0),
        )
    )

    rendered_lines = (
        output.getvalue()
        .removeprefix("\x1b[H")
        .removesuffix("\x1b[J")
        .split("\n")
    )
    assert rendered_lines
    assert all(line.startswith("\x1b[2K") for line in rendered_lines)


def test_tui_render_shows_fault_code_in_protocol_hex_and_decimal() -> None:
    output = io.StringIO()
    terminal = PosixAnsiTerminal(stdout=output)

    terminal.render(
        RuntimeSnapshot(
            mode="observe-can",
            feedback=PumpFeedback(1.0, -3, 0x52, 0),
        )
    )

    assert "故障码: 0x52 (82)" in output.getvalue()


def test_tui_render_shows_actual_and_desired_interlock_state() -> None:
    output = io.StringIO()
    terminal = PosixAnsiTerminal(stdout=output)

    terminal.render(
        RuntimeSnapshot(
            mode="move",
            command=PumpCommand.hydraulic_hold(),
            desired_command=PumpCommand.safe_stop(),
        )
    )

    rendered = output.getvalue()
    assert "实际输出: 互锁=开 PWM=0" in rendered
    assert "期望输出: 互锁=关 PWM=0" in rendered


class TtyFdStream:
    def __init__(self, descriptor: int) -> None:
        self.descriptor = descriptor
        self.fallback_writes: list[str] = []

    def isatty(self) -> bool:
        return True

    def fileno(self) -> int:
        return self.descriptor

    def write(self, payload: str) -> None:
        self.fallback_writes.append(payload)

    def flush(self) -> None:
        pass


def install_fake_posix_terminal_modules(monkeypatch, actions: list[tuple]) -> None:
    termios = ModuleType("termios")
    termios.TCSANOW = 0  # type: ignore[attr-defined]
    termios.TCSADRAIN = 1  # type: ignore[attr-defined]
    termios.tcgetattr = lambda descriptor: ("attrs", descriptor)  # type: ignore[attr-defined]
    termios.tcsetattr = lambda descriptor, mode, attrs: actions.append(  # type: ignore[attr-defined]
        ("termios", descriptor, mode, attrs)
    )
    tty = ModuleType("tty")
    tty.setcbreak = lambda descriptor: actions.append(("cbreak", descriptor))  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "termios", termios)
    monkeypatch.setitem(sys.modules, "tty", tty)
    monkeypatch.setattr(
        operator_runtime,
        "validate_foreground_terminal",
        lambda **_kwargs: None,
    )


def test_posix_terminal_drops_blocked_frame_and_restores_terminal(monkeypatch) -> None:
    actions: list[tuple] = []
    payloads: list[bytes] = []
    install_fake_posix_terminal_modules(monkeypatch, actions)
    monkeypatch.setattr(operator_runtime.os, "get_blocking", lambda _descriptor: True)
    monkeypatch.setattr(
        operator_runtime.os,
        "set_blocking",
        lambda descriptor, blocking: actions.append(("blocking", descriptor, blocking)),
    )

    def write(descriptor: int, payload: bytes) -> int:
        payloads.append(payload)
        actions.append(("write", descriptor))
        if len(payloads) == 2:
            raise BlockingIOError
        return len(payload)

    monkeypatch.setattr(operator_runtime.os, "write", write)
    terminal = PosixAnsiTerminal(stdin=TtyFdStream(10), stdout=TtyFdStream(11))

    terminal.open()
    terminal.render(RuntimeSnapshot(mode="monitor"))
    terminal.close()

    assert terminal.dropped_frames == 1
    assert ("blocking", 11, False) in actions
    assert actions[-1] == ("blocking", 11, True)
    termios_actions = [action for action in actions if action[0] == "termios"]
    assert termios_actions == [("termios", 10, 0, ("attrs", 10))]
    assert payloads[0] == b"\x1b[?25l\x1b[2J"
    assert payloads[-1] == b"\x1b[?25h\n"


def test_posix_terminal_drops_partial_frame_and_next_render_is_complete(
    monkeypatch,
) -> None:
    actions: list[tuple] = []
    payloads: list[bytes] = []
    install_fake_posix_terminal_modules(monkeypatch, actions)
    monkeypatch.setattr(operator_runtime.os, "get_blocking", lambda _descriptor: True)
    monkeypatch.setattr(operator_runtime.os, "set_blocking", lambda *_args: None)

    def write(_descriptor: int, payload: bytes) -> int:
        payloads.append(payload)
        if len(payloads) == 2:
            return 5
        return len(payload)

    monkeypatch.setattr(operator_runtime.os, "write", write)
    terminal = PosixAnsiTerminal(stdin=TtyFdStream(10), stdout=TtyFdStream(11))
    terminal.open()

    terminal.render(RuntimeSnapshot(mode="first"))
    terminal.render(RuntimeSnapshot(mode="second"))
    terminal.close()

    assert terminal.dropped_frames == 1
    assert payloads[2].startswith(b"\x1b[H")
    assert payloads[2].endswith(b"\x1b[J")
    assert "模式: second" in payloads[2].decode("utf-8")


class ScriptedSource:
    def __init__(self, *, open_result=True, error=None):
        self.open_result = open_result
        self.error = error
        self.closed = 0
        self.reads = 0

    def open(self):
        return self.open_result

    def read_sample(self):
        self.reads += 1
        if self.error:
            raise self.error
        return HeightSample(1.0, 4, 5.0, True, None)

    def close(self):
        self.closed += 1


def test_sensor_worker_exposes_read_exception_and_closes_idempotently() -> None:
    source = ScriptedSource(error=RuntimeError("串口断开"))
    worker = SensorWorker(source, poll_period_s=0.001, sleeper=lambda _s: None)
    worker.start()
    worker.join(timeout=1)

    assert "串口断开" in (worker.error or "")
    worker.stop()
    worker.stop()
    assert source.closed == 1


def test_sensor_worker_latches_open_failure() -> None:
    source = ScriptedSource(open_result=False)
    worker = SensorWorker(source, poll_period_s=0.01)
    worker.start()
    worker.join(timeout=1)

    assert "打开" in (worker.error or "")
    assert source.closed == 1


def test_shutdown_latch_keeps_first_reason_and_is_thread_safe() -> None:
    latch = ShutdownLatch()
    threads = [threading.Thread(target=latch.request, args=(str(i),)) for i in range(5)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert latch.requested
    assert latch.reason in {str(i) for i in range(5)}


def test_csv_logger_writes_complete_header_cycle_event_and_real_escaping(tmp_path) -> None:
    clock = FakeClock(12.5)
    logger = CsvEventLogger(tmp_path, "move", clock=clock, wall_clock=lambda: 99.0)
    snapshot = RuntimeSnapshot(
        mode="move",
        sample=HeightSample(12.4, 123, 45.5, True, '含逗号,与"引号"'),
        feedback=PumpFeedback(12.3, 22, 7, 8),
        target_mm=50.0,
        controller_state="coarse_lift",
        command=PumpCommand(interlock=True, lift_pwm=55, accel=1, decel=2),
        desired_command=PumpCommand(interlock=True, lift_pwm=60),
        zero_requested=False,
        lift_authorized=True,
        lower_authorized=False,
        lift_remaining_ms=650,
        lower_remaining_ms=0,
        controller_fault="故障,详情",
        pump_fault="CAN反馈超时",
    )
    logger.log("cycle", snapshot, detail='a,b"c')
    logger.log("authorization", snapshot, operator_key="u")
    path = logger.path
    logger.close()
    logger.close()

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == CSV_FIELDS
    assert rows[0]["sample_error"] == '含逗号,与"引号"'
    assert rows[0]["detail"] == 'a,b"c'
    assert rows[0]["command_lift_pwm"] == "55"
    assert rows[0]["desired_lift_pwm"] == "60"
    assert rows[0]["zero_requested"] == "False"
    assert rows[0]["pump_fault"] == "CAN反馈超时"
    assert [row["event"] for row in rows] == ["cycle", "authorization"]


def _lift_result() -> LiftCalibrationResult:
    trials = tuple(
        LiftTrial(
            pwm=40,
            repeat=repeat,
            start_delay_s=0.05,
            displacement_mm=4.0,
            speed_mm_s=40.0,
            coast_mm=0.5,
            peak_current_raw=140 + repeat,
            direction_consistent=True,
            success=True,
        )
        for repeat in range(1, 4)
    )
    return LiftCalibrationResult(40, 40, 0.05, 0.5, {40: 143}, trials)


def test_calibration_draft_roundtrip_preserves_complete_trials(tmp_path) -> None:
    store = CalibrationDraftStore(tmp_path / "lift-draft.json")
    expected = _lift_result()

    store.save_lift(expected)

    assert store.load_lift() == expected
    raw = json.loads(store.path.read_text(encoding="utf-8"))
    assert raw["schema_version"] == 3
    assert len(raw["lift"]["trials"]) == 3
    assert raw["lift"]["peak_current_by_pwm"] == {"40": 143}


@pytest.mark.parametrize("old_version", [1, 2])
def test_calibration_draft_rejects_old_lift_semantics(
    tmp_path, old_version: int
) -> None:
    path = tmp_path / "lift-draft.json"
    store = CalibrationDraftStore(path)
    store.save_lift(_lift_result())
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["schema_version"] = old_version
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="旧版.*重新执行起升标定"):
        store.load_lift()


def test_calibration_draft_rejects_noncanonical_peak_current_pwm_keys(tmp_path) -> None:
    path = tmp_path / "lift-draft.json"
    store = CalibrationDraftStore(path)
    store.save_lift(_lift_result())
    raw = json.loads(path.read_text(encoding="utf-8"))
    peaks = raw["lift"]["peak_current_by_pwm"]
    peaks["040"] = peaks.pop("40")
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="peak_current_by_pwm"):
        store.load_lift()


def _lower_result() -> LowerCalibrationResult:
    return LowerCalibrationResult(
        min_start_valve=0x10,
        comfortable_valve=None,
        trials=tuple(
            LowerTrial(
                valve=valve,
                displacement_mm=2.0,
                response_delay_s=0.1,
                direction_consistent=True,
                success=True,
            )
            for valve in LOWER_VALVE_LEVELS
        ),
    )


def _bundle() -> CalibrationBundle:
    return CalibrationBundle(
        min_stable_pwm=40,
        coarse_pwm=60,
        response_delay_s=0.1,
        max_coast_mm=0.5,
        peak_current_by_pwm={pwm: 100 + pwm for pwm in LIFT_PWM_LEVELS},
        lower_min_start_valve=0x10,
        lower_comfortable_valve=0x50,
    )


def test_lower_draft_roundtrip_preserves_all_trials_without_comfort_guess(tmp_path) -> None:
    store = LowerCalibrationDraftStore(tmp_path / "lower-draft.json")
    fingerprint = lift_calibration_fingerprint(_lift_result())

    store.save(_lower_result(), lift_fingerprint=fingerprint)
    loaded = store.load()

    assert loaded.result == _lower_result()
    assert loaded.result.comfortable_valve is None
    assert loaded.lift_fingerprint == fingerprint


def test_old_lower_draft_without_lift_fingerprint_is_explicitly_rejected(tmp_path) -> None:
    path = tmp_path / "lower-draft.json"
    store = LowerCalibrationDraftStore(path)
    store.save(
        _lower_result(),
        lift_fingerprint=lift_calibration_fingerprint(_lift_result()),
    )
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["schema_version"] = 1
    raw["lower"].pop("lift_fingerprint")
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="旧版.*起升指纹"):
        store.load()


def test_survey_draft_roundtrip_binds_recommendation_to_calibration_fingerprint(tmp_path) -> None:
    bundle = _bundle()
    draft = SurveyDraft(
        highest_observed_mm=1000.0,
        suggested_soft_limit_mm=950.0,
        temporary_max_height_mm=1200.0,
        calibration_fingerprint=calibration_fingerprint(bundle),
    )
    store = SurveyDraftStore(tmp_path / "survey-draft.json")

    store.save(draft)

    assert store.load() == draft


@pytest.mark.parametrize(
    "payload",
    [
        "not-json",
        json.dumps({"schema_version": 99, "lift": {}}),
        json.dumps({"schema_version": 1, "lift": {}, "extra": 1}),
    ],
)
def test_calibration_draft_rejects_corruption_and_unknown_fields(tmp_path, payload) -> None:
    path = tmp_path / "draft.json"
    path.write_text(payload, encoding="utf-8")

    with pytest.raises(CalibrationError):
        CalibrationDraftStore(path).load_lift()


class RecordingLockBackend:
    def __init__(self):
        self.held = False
        self.releases = 0

    def acquire(self, _stream):
        if self.held:
            raise BlockingIOError
        self.held = True

    def release(self, _stream):
        self.held = False
        self.releases += 1


def test_single_instance_lock_refuses_second_instance_and_release_is_idempotent(tmp_path) -> None:
    backend = RecordingLockBackend()
    first = SingleInstanceLock(tmp_path / "app.lock", backend=backend)
    second = SingleInstanceLock(tmp_path / "app.lock", backend=backend)
    first.acquire()
    with pytest.raises(RuntimeError, match="实例"):
        second.acquire()
    first.release()
    first.release()
    assert backend.releases == 1
