import json

import pytest

from agv_lift_height_control import ConfigError, load_config


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
        "can": {},
        "control": {},
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
