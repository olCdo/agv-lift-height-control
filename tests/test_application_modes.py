import json
from argparse import Namespace
from pathlib import Path

import pytest

from agv_lift_height_control import HeightSample, PumpCommand
from agv_lift_height_control.application import ApplicationDependencies, run_application
from agv_lift_height_control.calibration import CalibrationError
from agv_lift_height_control.operator_runtime import TerminalEvent


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
        stdout=None,
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
