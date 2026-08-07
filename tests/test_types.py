import pytest

from agv_lift_height_control import HeightSample, PumpCommand, PumpFeedback


def test_shared_dataclasses_hold_required_values() -> None:
    sample = HeightSample(timestamp=1.5, raw_count=120, height_mm=5.0, valid=True, error=None)
    feedback = PumpFeedback(timestamp=2.0, current_raw=13, fault_code=0, lower_current_raw=7)

    assert sample.valid is True
    assert feedback.lower_current_raw == 7


def test_pump_command_defaults_and_safe_stop() -> None:
    assert PumpCommand() == PumpCommand(False, 0, 0, 0, 0)
    assert PumpCommand.safe_stop() == PumpCommand(False, 0, 0, 0, 0)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("interlock", 1),
        ("lift_pwm", True),
        ("lift_pwm", -1),
        ("lift_pwm", 101),
        ("accel", 1.0),
        ("accel", 256),
        ("decel", True),
        ("lower_valve", -1),
    ],
)
def test_pump_command_rejects_invalid_field_values(field: str, value: object) -> None:
    values: dict[str, object] = {
        "interlock": False,
        "lift_pwm": 0,
        "accel": 0,
        "decel": 0,
        "lower_valve": 0,
    }
    values[field] = value

    with pytest.raises((TypeError, ValueError)):
        PumpCommand(**values)  # type: ignore[arg-type]
