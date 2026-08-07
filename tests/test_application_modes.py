import json
import io
from argparse import Namespace
from pathlib import Path

import pytest

from agv_lift_height_control import HeightSample, PumpCommand
from agv_lift_height_control.application import (
    ApplicationDependencies,
    _build_control_source,
    run_application,
)
from agv_lift_height_control.config import load_config
from agv_lift_height_control.calibration import (
    LIFT_PWM_LEVELS,
    LOWER_VALVE_LEVELS,
    CalibrationBundle,
    CalibrationError,
    CalibrationStore,
    LiftCalibrationResult,
    LiftTrial,
    LowerCalibrationResult,
    LowerTrial,
)
from agv_lift_height_control.operator_runtime import TerminalEvent
from agv_lift_height_control.runtime_storage import (
    CalibrationDraftStore,
    LowerCalibrationDraftStore,
    SurveyDraft,
    SurveyDraftStore,
    calibration_fingerprint,
    lift_calibration_fingerprint,
)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Terminal:
    def __init__(self, events):
        self.events = iter(events)

    def open(self):
        pass

    def read_event(self):
        return next(self.events, None)

    def render(self, _snapshot):
        pass

    def close(self):
        pass


class Logger:
    def log(self, *_args, **_kwargs):
        pass

    def close(self):
        pass


class Lock:
    def __init__(self):
        self.acquired = 0
        self.released = 0

    def acquire(self):
        self.acquired += 1

    def release(self):
        self.released += 1


class Worker:
    latest_sample = HeightSample(0.0, 1, 100.0, True, None)
    error = None

    def start(self):
        pass

    def stop(self):
        pass


class Pump:
    last_feedback = None
    thread_fault = None

    def __init__(self):
        self.commands = []

    def update_command(self, command):
        self.commands.append(command)

    def start(self):
        pass

    def stop(self):
        pass


class Observer:
    error = None

    def __init__(self):
        self.polls = 0

    def start(self):
        pass

    def poll(self):
        self.polls += 1
        return None

    def close(self):
        pass


def config_path(tmp_path):
    source = Path(__file__).parents[1] / "config" / "example.json"
    data = json.loads(source.read_text(encoding="utf-8"))
    data["storage"] = {
        "state_dir": str(tmp_path / "state"),
        "log_dir": str(tmp_path / "logs"),
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return str(path)


def harness(tmp_path, events):
    calls = []
    clock = Clock()
    pump = Pump()
    observer = Observer()
    lock = Lock()
    dependencies = ApplicationDependencies(
        clock=clock,
        sleeper=clock.sleep,
        source_factory=lambda _config: calls.append("source") or object(),
        worker_factory=lambda _source, _period: calls.append("worker") or Worker(),
        pump_factory=lambda _config: calls.append("pump") or pump,
        observer_factory=lambda _config: calls.append("observer") or observer,
        terminal_factory=lambda: Terminal(events),
        logger_factory=lambda _path, _mode: Logger(),
        lock_factory=lambda _path: lock,
        foreground_validator=lambda: calls.append("foreground"),
        signal_installer=lambda _latch: None,
        stdout=io.StringIO(),
    )
    return dependencies, calls, pump, observer, lock


def arguments(tmp_path, command, **kwargs):
    defaults = {
        "config": config_path(tmp_path),
        "command": command,
        "duration_s": 60.0,
        "comfortable_valve": 0x50,
        "target_mm": 100.0,
        "temporary_max_mm": 500.0,
        "confirm_save": False,
        "soft_limit_mm": 900.0,
    }
    defaults.update(kwargs)
    return Namespace(**defaults)


def test_monitor_constructs_sensor_only_and_never_constructs_can(tmp_path) -> None:
    deps, calls, _pump, _observer, lock = harness(
        tmp_path, [TerminalEvent.keypress("q")]
    )

    assert run_application(arguments(tmp_path, "monitor"), dependencies=deps) == 0

    assert calls == ["foreground", "source", "worker"]
    assert lock.acquired == lock.released == 1


def test_observe_can_constructs_passive_observer_without_pump_or_sensor(tmp_path) -> None:
    deps, calls, _pump, observer, _lock = harness(
        tmp_path, [None, TerminalEvent.keypress("q")]
    )

    run_application(arguments(tmp_path, "observe-can"), dependencies=deps)

    assert calls == ["foreground", "observer"]
    assert observer.polls == 1


def test_zero_can_constructs_only_pump_and_all_desired_commands_are_zero(tmp_path) -> None:
    deps, calls, pump, _observer, _lock = harness(
        tmp_path, [None, TerminalEvent.keypress("q")]
    )

    run_application(arguments(tmp_path, "zero-can", duration_s=5.0), dependencies=deps)

    assert calls == ["foreground", "pump"]
    assert pump.commands
    assert set(pump.commands) == {PumpCommand.safe_stop()}


def test_calibrate_lower_requires_valid_lift_draft_before_hardware_factories(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(
        tmp_path, [TerminalEvent.keypress("q")]
    )

    with pytest.raises(CalibrationError, match="草稿"):
        run_application(arguments(tmp_path, "calibrate-lower"), dependencies=deps)

    assert calls == ["foreground"]


def test_move_requires_final_bundle_before_hardware_factories(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(
        tmp_path, [TerminalEvent.keypress("q")]
    )

    with pytest.raises(CalibrationError, match="标定文件"):
        run_application(arguments(tmp_path, "move"), dependencies=deps)

    assert calls == ["foreground"]


def lift_result(*, current_offset=0) -> LiftCalibrationResult:
    trials = tuple(
        LiftTrial(
            pwm=pwm,
            repeat=repeat,
            start_delay_s=0.1,
            displacement_mm=2.0,
            speed_mm_s=6.0,
            coast_mm=0.5,
            peak_current_raw=100 + pwm + current_offset,
            direction_consistent=True,
            success=True,
        )
        for pwm in LIFT_PWM_LEVELS
        for repeat in range(1, 4)
    )
    return LiftCalibrationResult(
        40,
        60,
        0.1,
        0.5,
        {p: 100 + p + current_offset for p in LIFT_PWM_LEVELS},
        trials,
    )


def lower_result() -> LowerCalibrationResult:
    return LowerCalibrationResult(
        min_start_valve=0x10,
        comfortable_valve=None,
        trials=tuple(
            LowerTrial(valve, 2.0, 0.1, True, True)
            for valve in LOWER_VALVE_LEVELS
        ),
    )


def final_bundle(*, soft_limit=800.0) -> CalibrationBundle:
    return CalibrationBundle(
        40,
        60,
        0.1,
        0.5,
        {pwm: 100 + pwm for pwm in LIFT_PWM_LEVELS},
        0x10,
        0x50,
        soft_limit,
    )


def test_confirm_lower_is_hardware_free_and_uses_saved_successful_trial(tmp_path) -> None:
    deps, calls, _pump, _observer, lock = harness(tmp_path, [])
    state = tmp_path / "state"
    lift = lift_result()
    CalibrationDraftStore(state / "lift-calibration-draft.json").save_lift(lift)
    LowerCalibrationDraftStore(state / "lower-calibration-draft.json").save(
        lower_result(), lift_fingerprint=lift_calibration_fingerprint(lift)
    )

    run_application(
        arguments(tmp_path, "confirm-lower", comfortable_valve=0x50),
        dependencies=deps,
    )

    assert calls == []
    assert lock.acquired == lock.released == 1
    assert CalibrationStore(state / "calibration.json").load().lower_comfortable_valve == 0x50


def test_confirm_lower_rejects_when_lift_draft_changed_after_lower_measurement(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(tmp_path, [])
    state = tmp_path / "state"
    lift_store = CalibrationDraftStore(state / "lift-calibration-draft.json")
    lift_a = lift_result(current_offset=0)
    lift_store.save_lift(lift_a)
    LowerCalibrationDraftStore(state / "lower-calibration-draft.json").save(
        lower_result(), lift_fingerprint=lift_calibration_fingerprint(lift_a)
    )
    lift_store.save_lift(lift_result(current_offset=1))

    with pytest.raises(CalibrationError, match="起升标定.*不匹配"):
        run_application(
            arguments(tmp_path, "confirm-lower", comfortable_valve=0x50),
            dependencies=deps,
        )

    assert calls == []
    assert not (state / "calibration.json").exists()


def test_confirm_upper_is_hardware_free_and_rejects_value_above_safe_suggestion(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(tmp_path, [])
    state = tmp_path / "state"
    bundle = final_bundle(soft_limit=None)
    CalibrationStore(state / "calibration.json").save(bundle)
    SurveyDraftStore(state / "upper-survey-draft.json").save(
        SurveyDraft(1000.0, 950.0, 1200.0, calibration_fingerprint(bundle))
    )

    with pytest.raises(CalibrationError, match="建议"):
        run_application(
            arguments(tmp_path, "confirm-upper", soft_limit_mm=951.0),
            dependencies=deps,
        )
    assert calls == []

    run_application(
        arguments(tmp_path, "confirm-upper", soft_limit_mm=900.0),
        dependencies=deps,
    )
    assert CalibrationStore(state / "calibration.json").load().soft_upper_limit_mm == 900.0


def test_calibrate_lift_effective_limit_is_minimum_of_temporary_persistent_and_absolute(
    tmp_path,
) -> None:
    config_file = config_path(tmp_path)
    config = load_config(config_file)
    store = CalibrationStore(tmp_path / "state" / "calibration.json")
    store.save(final_bundle(soft_limit=800.0))

    source = _build_control_source(
        Namespace(command="calibrate-lift", temporary_max_mm=1000.0),
        config,
        store,
    )

    assert source.session._absolute_max_height_mm == 800.0


def test_calibrate_lift_missing_gate_refuses_before_hardware_factories(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(tmp_path, [])

    with pytest.raises(CalibrationError, match="临时最大高度"):
        run_application(
            arguments(tmp_path, "calibrate-lift", temporary_max_mm=None),
            dependencies=deps,
        )

    assert calls == ["foreground"]
