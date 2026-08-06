import json

import pytest

import agv_lift_height_control as package
from agv_lift_height_control import ConfigError, ControlConfig, SensorConfig, load_config


def valid_control_config() -> dict[str, object]:
    return {
        "tolerance_mm": 2.0,
        "stable_time_s": 0.5,
        "overshoot_limit_mm": 5.0,
        "absolute_max_height_mm": 2900.0,
        "max_speed_mm_s": 1200.0,
        "sensor_timeout_s": 0.1,
        "control_loop_timeout_s": 0.1,
        "current_multiplier": 1.5,
        "current_duration_s": 0.2,
        "direction_tolerance_mm": 1.0,
        "survey_max_on_s": 1.0,
        "survey_pause_s": 0.5,
    }


def valid_config() -> dict[str, object]:
    return {
        "sensor": {
            "transport": "rtu",
            "port": "/dev/ttyS7",
            "baudrate": 115200,
            "bytesize": 8,
            "parity": "N",
            "stopbits": 1,
            "timeout_s": 0.1,
            "slave_id": 3,
            "register_address": 0,
            "register_count": 2,
            "word_order": "high_low",
            "wheel_circumference_mm": 200.0,
            "counts_per_revolution": 4096,
            "range_mm": 3000.0,
            "poll_period_s": 0.02,
        },
        "can": {
            "interface": "can0",
            "bitrate": 500000,
            "command_id": 0x217,
            "feedback_id": 0x197,
            "nmt_id": 0,
            "send_period_s": 0.05,
            "feedback_timeout_s": 0.15,
            "command_timeout_s": 0.15,
            "preflight_s": 0.3,
            "startup_nmt_s": 5.0,
            "shutdown_zero_frames": 3,
        },
        "control": valid_control_config(),
        "storage": {},
    }


def write_config(tmp_path, data: object):
    path = tmp_path / "config.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_load_config_returns_typed_sensor_settings(tmp_path) -> None:
    config = load_config(write_config(tmp_path, valid_config()))

    assert config.sensor.port == "/dev/ttyS7"
    assert config.sensor.word_order == "high_low"
    assert config.sensor.counts_per_revolution == 4096


def test_load_config_returns_typed_can_settings(tmp_path) -> None:
    config = load_config(write_config(tmp_path, valid_config()))

    assert hasattr(package, "CanConfig")
    assert isinstance(config.can, package.CanConfig)
    assert config.can.interface == "can0"
    assert config.can.command_id == 0x217
    assert config.can.feedback_id == 0x197
    assert config.can.shutdown_zero_frames == 3


def test_load_config_returns_strict_typed_control_settings(tmp_path) -> None:
    config = load_config(write_config(tmp_path, valid_config()))

    assert isinstance(config.control, ControlConfig)
    assert config.control.tolerance_mm == 2.0
    assert config.control.absolute_max_height_mm == 2900.0


def test_example_config_declares_safe_control_defaults() -> None:
    with open("config/example.json", encoding="utf-8") as stream:
        example = json.load(stream)

    assert example["control"] == valid_control_config()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda data: data.pop("can"),
        lambda data: data["sensor"].update({"transport": "tcp"}),
        lambda data: data["sensor"].update({"register_count": 1}),
        lambda data: data["sensor"].update({"timeout_s": 0}),
        lambda data: data["sensor"].update({"poll_period_s": float("inf")}),
        lambda data: data["sensor"].update({"word_order": "middle"}),
        lambda data: data["sensor"].update({"register_address": 65535}),
        lambda data: data["sensor"].update({"counts_per_revolution": True}),
        lambda data: data["sensor"].update({"range_mm": -1}),
        lambda data: data["can"].update({"bitrate": 250000}),
        lambda data: data["can"].update({"command_id": 0x800}),
        lambda data: data["can"].update({"feedback_id": 0x217}),
        lambda data: data["can"].update({"nmt_id": 1}),
        lambda data: data["can"].update({"send_period_s": 0}),
        lambda data: data["can"].update({"send_period_s": 0.049}),
        lambda data: data["can"].update({"send_period_s": 0.051}),
        lambda data: data["can"].update({"feedback_timeout_s": 0.151}),
        lambda data: data["can"].update({"command_timeout_s": 0.151}),
        lambda data: data["can"].update({"preflight_s": -1}),
        lambda data: data["can"].update({"startup_nmt_s": float("nan")}),
        lambda data: data["can"].update({"shutdown_zero_frames": 2}),
        lambda data: data["control"].update({"absolute_max_height_mm": 2900.1}),
        lambda data: data["control"].update({"max_speed_mm_s": True}),
        lambda data: data["control"].update({"sensor_timeout_s": float("nan")}),
        lambda data: data["control"].update({"current_multiplier": 1.0}),
        lambda data: data["control"].update({"survey_pause_s": 0}),
    ],
)
def test_load_config_rejects_missing_or_invalid_values(tmp_path, mutate) -> None:
    data = valid_config()
    mutate(data)

    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, data))


def test_load_config_rejects_malformed_json(tmp_path) -> None:
    path = tmp_path / "broken.json"
    path.write_text("{not json", encoding="utf-8")

    with pytest.raises(ConfigError, match="JSON"):
        load_config(path)


@pytest.mark.parametrize(
    ("mutate", "field_name"),
    [
        (lambda data: data.update({"unexpected_root": {}}), "unexpected_root"),
        (lambda data: data["sensor"].update({"slaev_id": 3}), "slaev_id"),
        (lambda data: data["can"].update({"bitrtae": 500000}), "bitrtae"),
        (lambda data: data["control"].update({"tolernace_mm": 2}), "tolernace_mm"),
    ],
)
def test_load_config_rejects_unknown_fields_with_field_name(tmp_path, mutate, field_name: str) -> None:
    data = valid_config()
    mutate(data)

    with pytest.raises(ConfigError, match=field_name):
        load_config(write_config(tmp_path, data))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("transport", "tcp"),
        ("port", ""),
        ("register_count", 3),
        ("word_order", "middle"),
        ("counts_per_revolution", True),
        ("timeout_s", float("inf")),
    ],
)
def test_sensor_config_rejects_invalid_direct_construction(field: str, value: object) -> None:
    sensor = valid_config()["sensor"]
    assert isinstance(sensor, dict)
    sensor[field] = value

    with pytest.raises(ConfigError, match=field):
        SensorConfig(**sensor)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("interface", ""),
        ("interface", "接口接口接口"),
        ("bitrate", True),
        ("bitrate", 250000),
        ("command_id", 0x197),
        ("feedback_id", 0x217),
        ("nmt_id", 1),
        ("send_period_s", 0.049),
        ("send_period_s", 0.051),
        ("send_period_s", 0.2),
        ("feedback_timeout_s", 0),
        ("command_timeout_s", 0.151),
        ("preflight_s", float("inf")),
        ("startup_nmt_s", -1),
        ("shutdown_zero_frames", 2),
    ],
)
def test_can_config_rejects_invalid_direct_construction(field: str, value: object) -> None:
    can = valid_config()["can"]
    assert isinstance(can, dict)
    can[field] = value

    assert hasattr(package, "CanConfig")
    with pytest.raises(ConfigError, match=field):
        package.CanConfig(**can)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("tolerance_mm", 0),
        ("stable_time_s", True),
        ("overshoot_limit_mm", float("inf")),
        ("absolute_max_height_mm", 2901),
        ("max_speed_mm_s", -1),
        ("sensor_timeout_s", 0),
        ("control_loop_timeout_s", 0),
        ("current_multiplier", 1.0),
        ("current_duration_s", 0),
        ("direction_tolerance_mm", 0),
        ("survey_max_on_s", 0),
        ("survey_pause_s", 0),
        ("tolerance_mm", 2.001),
        ("stable_time_s", 0.499),
        ("overshoot_limit_mm", 5.001),
        ("max_speed_mm_s", 1200.001),
        ("sensor_timeout_s", 0.1001),
        ("control_loop_timeout_s", 0.1001),
        ("current_multiplier", 1.5001),
        ("current_duration_s", 0.2001),
        ("direction_tolerance_mm", 2.001),
        ("survey_max_on_s", 1.001),
    ],
)
def test_control_config_rejects_invalid_direct_construction(
    field: str, value: object
) -> None:
    control = valid_control_config()
    control[field] = value

    with pytest.raises(ConfigError, match=field):
        ControlConfig(**control)  # type: ignore[arg-type]
