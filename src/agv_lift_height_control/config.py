"""JSON 配置加载与传感器、CAN、控制安全参数的严格校验。"""

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

    def __post_init__(self) -> None:
        """直接构造配置时也保留与 JSON 加载相同的生产约束。"""
        if type(self.transport) is not str or self.transport != "rtu":
            raise ConfigError("sensor.transport 在 v1 必须是 'rtu'")
        if type(self.port) is not str or not self.port:
            raise ConfigError("sensor.port 必须是非空字符串")
        if type(self.parity) is not str or self.parity not in {"N", "E", "O", "M", "S"}:
            raise ConfigError("sensor.parity 必须是 N/E/O/M/S")
        if type(self.word_order) is not str or self.word_order not in {"high_low", "low_high"}:
            raise ConfigError("sensor.word_order 必须是 high_low 或 low_high")
        _validate_integer_value("baudrate", self.baudrate, 1)
        _validate_integer_value("bytesize", self.bytesize, 5, 8)
        _validate_integer_value("stopbits", self.stopbits, 1, 2)
        _validate_integer_value("slave_id", self.slave_id, 1, 247)
        _validate_integer_value("register_address", self.register_address, 0, 65534)
        _validate_integer_value("register_count", self.register_count, 2, 2)
        _validate_integer_value("counts_per_revolution", self.counts_per_revolution, 1)
        _validate_number_value("timeout_s", self.timeout_s)
        _validate_number_value("wheel_circumference_mm", self.wheel_circumference_mm)
        _validate_number_value("range_mm", self.range_mm)
        _validate_number_value("poll_period_s", self.poll_period_s)


@dataclass(frozen=True)
class CanConfig:
    """CAN 泵协议和失效保护时间参数，单位均为秒或 bit/s。"""

    interface: str
    bitrate: int
    command_id: int
    feedback_id: int
    nmt_id: int
    send_period_s: float
    feedback_timeout_s: float
    command_timeout_s: float
    preflight_s: float
    startup_nmt_s: float
    shutdown_zero_frames: int

    def __post_init__(self) -> None:
        if (
            type(self.interface) is not str
            or not self.interface
            or len(self.interface.encode("utf-8")) > 15
            or any(character.isspace() for character in self.interface)
        ):
            raise ConfigError("can.interface 必须是 1..15 字节且不含空白的接口名")
        _validate_integer_value("bitrate", self.bitrate, 500000, 500000, section="can")
        _validate_integer_value("command_id", self.command_id, 0, 0x7FF, section="can")
        _validate_integer_value("feedback_id", self.feedback_id, 0, 0x7FF, section="can")
        _validate_integer_value("nmt_id", self.nmt_id, 0, 0x7FF, section="can")
        if self.command_id != 0x217:
            raise ConfigError("can.command_id 必须是标准帧 ID 0x217")
        if self.feedback_id != 0x197:
            raise ConfigError("can.feedback_id 必须是标准帧 ID 0x197")
        if self.nmt_id != 0:
            raise ConfigError("can.nmt_id 必须是 CANopen NMT ID 0x000")
        _validate_number_range("send_period_s", self.send_period_s, 0.05, 0.05)
        _validate_number_range("feedback_timeout_s", self.feedback_timeout_s, 0.001, 0.15)
        _validate_number_range("command_timeout_s", self.command_timeout_s, 0.001, 0.15)
        # 这些下限保证配置不能缩短协议要求的冲突探测和 NMT 安全窗口。
        _validate_number_range("preflight_s", self.preflight_s, 0.3, 10.0)
        _validate_number_range("startup_nmt_s", self.startup_nmt_s, 5.0, 60.0)
        _validate_integer_value(
            "shutdown_zero_frames",
            self.shutdown_zero_frames,
            3,
            100,
            section="can",
        )


@dataclass(frozen=True)
class ControlConfig:
    """高度闭环的安全阈值；长度单位为 mm，时间单位为秒。"""

    tolerance_mm: float
    stable_time_s: float
    overshoot_limit_mm: float
    absolute_max_height_mm: float
    max_speed_mm_s: float
    sensor_timeout_s: float
    control_loop_timeout_s: float
    current_multiplier: float
    current_duration_s: float
    direction_tolerance_mm: float
    survey_max_on_s: float
    survey_pause_s: float

    def __post_init__(self) -> None:
        for name in (
            "tolerance_mm",
            "stable_time_s",
            "overshoot_limit_mm",
            "max_speed_mm_s",
            "sensor_timeout_s",
            "control_loop_timeout_s",
            "current_duration_s",
            "direction_tolerance_mm",
            "survey_max_on_s",
            "survey_pause_s",
        ):
            _validate_number_value(name, getattr(self, name), section="control")
        upper_bounds = {
            "tolerance_mm": 2.0,
            "overshoot_limit_mm": 5.0,
            "max_speed_mm_s": 1200.0,
            "sensor_timeout_s": 0.1,
            "control_loop_timeout_s": 0.1,
            "current_duration_s": 0.2,
            "direction_tolerance_mm": 2.0,
            "survey_max_on_s": 1.0,
        }
        for name, maximum in upper_bounds.items():
            if getattr(self, name) > maximum:
                raise ConfigError(f"control.{name} 不得超过 {maximum}")
        if self.stable_time_s < 0.5:
            raise ConfigError("control.stable_time_s 不得小于 0.5")
        _validate_number_range(
            "absolute_max_height_mm",
            self.absolute_max_height_mm,
            0.001,
            2900.0,
            section="control",
        )
        multiplier = _validate_number_value(
            "current_multiplier", self.current_multiplier, section="control"
        )
        if not 1.0 < multiplier <= 1.5:
            raise ConfigError("control.current_multiplier 必须大于 1 且不超过 1.5")
        if self.tolerance_mm >= self.overshoot_limit_mm:
            raise ConfigError("control.tolerance_mm 必须小于 overshoot_limit_mm")


@dataclass(frozen=True)
class StorageConfig:
    """运行状态与 CSV 日志目录；加载配置只展开路径，不创建目录。"""

    state_dir: Path
    log_dir: Path

    def __post_init__(self) -> None:
        for name in ("state_dir", "log_dir"):
            value = getattr(self, name)
            if isinstance(value, Path):
                text = str(value)
            elif type(value) is str:
                text = value
            else:
                raise ConfigError(f"storage.{name} 必须是非空路径字符串")
            if not text.strip():
                raise ConfigError(f"storage.{name} 必须是非空路径字符串")
            object.__setattr__(self, name, Path(text).expanduser())


@dataclass(frozen=True)
class AppConfig:
    sensor: SensorConfig
    can: CanConfig
    control: ControlConfig
    storage: StorageConfig


ROOT_FIELDS = frozenset({"sensor", "can", "control", "storage"})
SENSOR_FIELDS = frozenset(SensorConfig.__dataclass_fields__)
CAN_FIELDS = frozenset(CanConfig.__dataclass_fields__)
CONTROL_FIELDS = frozenset(ControlConfig.__dataclass_fields__)
STORAGE_FIELDS = frozenset(StorageConfig.__dataclass_fields__)


def load_config(path: str | Path) -> AppConfig:
    """读取并严格校验 v1 配置；v1 只支持 Modbus RTU。"""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"无法读取 JSON 配置: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError("配置根节点必须是对象")
    _reject_unknown_fields(raw, ROOT_FIELDS, "根配置")

    sections: dict[str, dict[str, Any]] = {}
    for section in ("sensor", "can", "control", "storage"):
        value = raw.get(section)
        if not isinstance(value, dict):
            raise ConfigError(f"缺少或无效的 {section} 配置段")
        sections[section] = value
    return AppConfig(
        sensor=_parse_sensor(sections["sensor"]),
        can=_parse_can(sections["can"]),
        control=_parse_control(sections["control"]),
        storage=_parse_storage(sections["storage"]),
    )


def _parse_sensor(data: dict[str, Any]) -> SensorConfig:
    _reject_unknown_fields(data, SENSOR_FIELDS, "sensor 配置")
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


def _parse_can(data: dict[str, Any]) -> CanConfig:
    """逐字段读取 CAN 配置，使缺失字段也统一转换为可操作的 ConfigError。"""
    _reject_unknown_fields(data, CAN_FIELDS, "can 配置")
    return CanConfig(
        interface=_string(data, "interface", section="can"),
        bitrate=_integer(data, "bitrate", 0, 0x7FFFFFFF, section="can"),
        command_id=_integer(data, "command_id", 0, 0x7FF, section="can"),
        feedback_id=_integer(data, "feedback_id", 0, 0x7FF, section="can"),
        nmt_id=_integer(data, "nmt_id", 0, 0x7FF, section="can"),
        send_period_s=_number(data, "send_period_s", positive=True, section="can"),
        feedback_timeout_s=_number(data, "feedback_timeout_s", positive=True, section="can"),
        command_timeout_s=_number(data, "command_timeout_s", positive=True, section="can"),
        preflight_s=_number(data, "preflight_s", positive=True, section="can"),
        startup_nmt_s=_number(data, "startup_nmt_s", positive=True, section="can"),
        shutdown_zero_frames=_integer(data, "shutdown_zero_frames", 0, 100, section="can"),
    )


def _parse_control(data: dict[str, Any]) -> ControlConfig:
    """逐字段解析控制阈值，禁止拼写错误被静默忽略。"""
    _reject_unknown_fields(data, CONTROL_FIELDS, "control 配置")
    return ControlConfig(
        **{
            name: _number(data, name, positive=True, section="control")
            for name in CONTROL_FIELDS
        }
    )


def _parse_storage(data: dict[str, Any]) -> StorageConfig:
    """严格解析运行目录；路径的创建延迟到真正写入时。"""
    _reject_unknown_fields(data, STORAGE_FIELDS, "storage 配置")
    return StorageConfig(
        state_dir=_string(data, "state_dir", section="storage"),
        log_dir=_string(data, "log_dir", section="storage"),
    )


def _string(data: dict[str, Any], name: str, *, section: str = "sensor") -> str:
    value = data.get(name)
    if not isinstance(value, str):
        raise ConfigError(f"{section}.{name} 必须是字符串")
    return value


def _integer(
    data: dict[str, Any],
    name: str,
    minimum: int,
    maximum: int | None = None,
    *,
    section: str = "sensor",
) -> int:
    value = data.get(name)
    return _validate_integer_value(name, value, minimum, maximum, section=section)


def _number(data: dict[str, Any], name: str, *, positive: bool, section: str = "sensor") -> float:
    value = data.get(name)
    return _validate_number_value(name, value, positive=positive, section=section)


def _reject_unknown_fields(data: dict[str, Any], allowed: frozenset[str], context: str) -> None:
    unknown = sorted(set(data).difference(allowed))
    if unknown:
        raise ConfigError(f"{context} 包含未知字段: {', '.join(unknown)}")


def _validate_integer_value(
    name: str,
    value: object,
    minimum: int,
    maximum: int | None = None,
    *,
    section: str = "sensor",
) -> int:
    if type(value) is not int:
        raise ConfigError(f"{section}.{name} 必须是整数")
    if value < minimum or (maximum is not None and value > maximum):
        raise ConfigError(f"{section}.{name} 超出允许范围")
    return value


def _validate_number_value(
    name: str,
    value: object,
    *,
    positive: bool = True,
    section: str = "sensor",
) -> float:
    if type(value) not in {int, float}:
        raise ConfigError(f"{section}.{name} 必须是数字")
    result = float(value)
    if not isfinite(result) or (positive and result <= 0):
        raise ConfigError(f"{section}.{name} 必须大于零")
    return result


def _validate_number_range(
    name: str,
    value: object,
    minimum: float,
    maximum: float,
    *,
    section: str = "can",
) -> float:
    result = _validate_number_value(name, value, section=section)
    if not minimum <= result <= maximum:
        raise ConfigError(f"{section}.{name} 超出允许范围 {minimum}..{maximum}")
    return result
