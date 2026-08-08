"""高度控制模块的无硬件共享数据类型。"""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class HeightSample:
    timestamp: float
    raw_count: int | None
    height_mm: float | None
    valid: bool
    error: str | None


class HeightSource(Protocol):
    """高度传感器的统一生命周期和采样接口。"""

    def open(self) -> bool:
        """打开底层连接，连接成功时返回 True。"""

    def read_sample(self) -> HeightSample:
        """读取一帧高度；通信失败以无效样本表示。"""

    def close(self) -> None:
        """释放底层连接，可重复调用。"""


@dataclass(frozen=True)
class PumpCommand:
    interlock: bool = False
    lift_pwm: int = 0
    accel: int = 0
    decel: int = 0
    lower_valve: int = 0

    def __post_init__(self) -> None:
        if type(self.interlock) is not bool:
            raise TypeError("interlock 必须是 bool")
        self._validate_int("lift_pwm", self.lift_pwm, 0, 100)
        self._validate_int("accel", self.accel, 0, 255)
        self._validate_int("decel", self.decel, 0, 255)
        self._validate_int("lower_valve", self.lower_valve, 0, 255)

    @staticmethod
    def _validate_int(name: str, value: int, minimum: int, maximum: int) -> None:
        if type(value) is not int:
            raise TypeError(f"{name} 必须是整数，不能是 bool")
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} 必须在 {minimum}..{maximum} 范围内")

    @classmethod
    def hydraulic_hold(cls) -> "PumpCommand":
        """返回只启用互锁的正常保持命令；不得用于故障、超时或退出。"""
        return cls(interlock=True)

    @classmethod
    def safe_stop(cls) -> "PumpCommand":
        """返回不使能且所有输出为零的安全停机命令。"""
        return cls()


@dataclass(frozen=True)
class PumpFeedback:
    timestamp: float
    current_raw: int
    fault_code: int
    lower_current_raw: int
