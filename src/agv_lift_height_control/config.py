"""JSON 配置加载和 v1 RTU 传感器参数校验。"""

import json
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """配置文件语法或字段约束不满足时抛出。"""


@dataclass(frozen=True)
class SensorConfig:
    transport: str
    port: str
    baudrate: int
    bytesize: int
    parity: str
    stopbits: int
    timeout_s: float
    slave_id: int
    register_address: int
    register_count: int
    word_order: str
    wheel_circumference_mm: float
    counts_per_revolution: int
    range_mm: float
    poll_period_s: float


@dataclass(frozen=True)
class AppConfig:
    sensor: SensorConfig
    can: dict[str, Any]
    control: dict[str, Any]
    storage: dict[str, Any]


def load_config(path: str | Path) -> AppConfig:
    """读取并严格校验 v1 配置；v1 只支持 Modbus RTU。"""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"无法读取 JSON 配置: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("配置根节点必须是对象")

    sections: dict[str, dict[str, Any]] = {}
    for section in ("sensor", "can", "control", "storage"):
        value = raw.get(section)
        if not isinstance(value, dict):
            raise ConfigError(f"缺少或无效的 {section} 配置段")
        sections[section] = value
    return AppConfig(
        sensor=_parse_sensor(sections["sensor"]),
        can=sections["can"],
        control=sections["control"],
        storage=sections["storage"],
    )


def _parse_sensor(data: dict[str, Any]) -> SensorConfig:
    transport = _string(data, "transport")
    if transport != "rtu":
        raise ConfigError("v1 仅支持 sensor.transport='rtu'")
    port = _string(data, "port")
    if not port:
        raise ConfigError("sensor.port 不能为空")
    parity = _string(data, "parity")
    if parity not in {"N", "E", "O", "M", "S"}:
        raise ConfigError("sensor.parity 必须是 N/E/O/M/S")
    word_order = _string(data, "word_order")
    if word_order not in {"high_low", "low_high"}:
        raise ConfigError("sensor.word_order 必须是 high_low 或 low_high")

    baudrate = _integer(data, "baudrate", 1)
    bytesize = _integer(data, "bytesize", 5, 8)
    stopbits = _integer(data, "stopbits", 1, 2)
    slave_id = _integer(data, "slave_id", 1, 247)
    # FC03 固定读取两个连续寄存器，起始地址必须保留第二个 16 位地址。
    register_address = _integer(data, "register_address", 0, 65534)
    register_count = _integer(data, "register_count", 2, 2)
    counts_per_revolution = _integer(data, "counts_per_revolution", 1)
    timeout_s = _number(data, "timeout_s", positive=True)
    wheel_circumference_mm = _number(data, "wheel_circumference_mm", positive=True)
    range_mm = _number(data, "range_mm", positive=True)
    poll_period_s = _number(data, "poll_period_s", positive=True)
    return SensorConfig(
        transport=transport,
        port=port,
        baudrate=baudrate,
        bytesize=bytesize,
        parity=parity,
        stopbits=stopbits,
        timeout_s=timeout_s,
        slave_id=slave_id,
        register_address=register_address,
        register_count=register_count,
        word_order=word_order,
        wheel_circumference_mm=wheel_circumference_mm,
        counts_per_revolution=counts_per_revolution,
        range_mm=range_mm,
        poll_period_s=poll_period_s,
    )


def _string(data: dict[str, Any], name: str) -> str:
    value = data.get(name)
    if not isinstance(value, str):
        raise ConfigError(f"sensor.{name} 必须是字符串")
    return value


def _integer(data: dict[str, Any], name: str, minimum: int, maximum: int | None = None) -> int:
    value = data.get(name)
    if type(value) is not int:
        raise ConfigError(f"sensor.{name} 必须是整数")
    if value < minimum or (maximum is not None and value > maximum):
        raise ConfigError(f"sensor.{name} 超出允许范围")
    return value


def _number(data: dict[str, Any], name: str, *, positive: bool) -> float:
    value = data.get(name)
    if type(value) not in {int, float}:
        raise ConfigError(f"sensor.{name} 必须是数字")
    result = float(value)
    if not isfinite(result) or (positive and result <= 0):
        raise ConfigError(f"sensor.{name} 必须大于零")
    return result
