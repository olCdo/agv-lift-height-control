"""observe-can 使用的严格只读 0x197 观察器。"""

from __future__ import annotations

from collections.abc import Callable
from time import monotonic
from typing import Any

from .can_pump import inspect_can_link, parse_pump_feedback
from .config import CanConfig
from .types import PumpFeedback

MAX_FRAMES_PER_POLL = 64
MAX_SECONDS_PER_POLL = 0.002


class PassiveCanObserver:
    """仅打开 SocketCAN 接收并解析 0x197；本类没有 send 路径。"""

    def __init__(
        self,
        config: CanConfig,
        *,
        bus_factory: Callable[[str], Any] | None = None,
        link_checker: Callable[[str, int], Any] = inspect_can_link,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        self._config = config
        self._bus_factory = bus_factory or _create_receive_bus
        self._link_checker = link_checker
        self._clock = clock
        self._bus: Any | None = None
        self.last_feedback: PumpFeedback | None = None
        self.error: str | None = None

    def start(self) -> None:
        if self._bus is not None:
            return
        self._link_checker(self._config.interface, self._config.bitrate)
        self._bus = self._bus_factory(self._config.interface)

    def poll(self) -> PumpFeedback | None:
        bus = self._bus
        if bus is None:
            raise RuntimeError("只读 CAN 观察器尚未启动")
        try:
            deadline = monotonic() + MAX_SECONDS_PER_POLL
            for _ in range(MAX_FRAMES_PER_POLL):
                frame = bus.recv(timeout=0)
                if frame is None:
                    return self.last_feedback
                # 协议只允许固定标准帧 0x197；配置层也已把 feedback_id 锁死。
                if getattr(frame, "arbitration_id", None) != 0x197:
                    if monotonic() >= deadline:
                        return self.last_feedback
                    continue
                self.last_feedback = parse_pump_feedback(frame, timestamp=self._clock())
                return self.last_feedback
            # 持续噪声也只能占用一个有界周期，下一主循环仍可处理信号和安全归零。
            return self.last_feedback
        except Exception as exc:
            self.error = f"只读 CAN 观察失败: {exc}"
            raise

    def close(self) -> None:
        bus = self._bus
        if bus is None:
            return
        self._bus = None
        shutdown = getattr(bus, "shutdown", None)
        if callable(shutdown):
            shutdown()


def _create_receive_bus(interface: str) -> Any:
    import can

    common = {"interface": "socketcan", "channel": interface}
    filters = [{"can_id": 0x197, "can_mask": 0x7FF, "extended": False}]
    try:
        return can.interface.Bus(**common, can_filters=filters)
    except TypeError:
        # 旧版 python-can 可能不接受构造期过滤器；解析层仍严格拒绝非 0x197。
        return can.interface.Bus(**common)
