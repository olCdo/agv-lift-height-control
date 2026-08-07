import csv
import io
import json
import threading

import pytest

import agv_lift_height_control as package
from agv_lift_height_control import HeightSample, PumpCommand, PumpFeedback
from agv_lift_height_control.calibration import (
    LIFT_PWM_LEVELS,
    CalibrationError,
    LiftCalibrationResult,
    LiftTrial,
)
from agv_lift_height_control.operator_runtime import (
    CSV_FIELDS,
    EOF_EVENT,
    CsvEventLogger,
    DeadmanAuthorizer,
    RuntimeSnapshot,
    SensorWorker,
    ShutdownLatch,
    SingleInstanceLock,
    TerminalEvent,
    validate_foreground_terminal,
)
from agv_lift_height_control.runtime_storage import CalibrationDraftStore


def test_runtime_public_interfaces_are_exported_from_package() -> None:
    for name in (
        "StorageConfig",
        "DeadmanAuthorizer",
        "SensorWorker",
        "CsvEventLogger",
        "CalibrationDraftStore",
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
        lift_authorized=True,
        lower_authorized=False,
        lift_remaining_ms=650,
        lower_remaining_ms=0,
        controller_fault="故障,详情",
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
    assert [row["event"] for row in rows] == ["cycle", "authorization"]


def _lift_result() -> LiftCalibrationResult:
    trials = tuple(
        LiftTrial(
            pwm=pwm,
            repeat=repeat,
            start_delay_s=0.1,
            displacement_mm=2.0,
            speed_mm_s=6.0,
            coast_mm=0.5,
            peak_current_raw=100 + pwm,
            direction_consistent=True,
            success=True,
        )
        for pwm in LIFT_PWM_LEVELS
        for repeat in range(1, 4)
    )
    return LiftCalibrationResult(40, 60, 0.1, 0.5, {p: 100 + p for p in LIFT_PWM_LEVELS}, trials)


def test_calibration_draft_roundtrip_preserves_complete_trials(tmp_path) -> None:
    store = CalibrationDraftStore(tmp_path / "lift-draft.json")
    expected = _lift_result()

    store.save_lift(expected)

    assert store.load_lift() == expected


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
