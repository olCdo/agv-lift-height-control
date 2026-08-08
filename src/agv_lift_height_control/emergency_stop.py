"""线程安全的锁存急停门禁。"""

from collections.abc import Callable
from dataclasses import dataclass
from threading import RLock
from time import monotonic

from .types import PumpCommand


@dataclass(frozen=True)
class EmergencyStopSnapshot:
    """急停状态的不可变快照，可安全交给其他线程读取。"""

    active: bool
    reason: str | None
    triggered_at: float | None
    zero_sent_after_trigger: bool
    transport_fault: str | None


class EmergencyStopLatch:
    """锁存首次急停，并用全零发送证据和传输状态约束解除操作。"""

    def __init__(self, clock: Callable[[], float] = monotonic) -> None:
        self._clock = clock
        self._lock = RLock()
        self._active = False
        self._reason: str | None = None
        self._triggered_at: float | None = None
        self._zero_sent_after_trigger = False
        self._transport_fault: str | None = None

    def snapshot(self) -> EmergencyStopSnapshot:
        """在同一把锁内复制全部字段，避免读到跨状态转换的混合值。"""
        with self._lock:
            return EmergencyStopSnapshot(
                active=self._active,
                reason=self._reason,
                triggered_at=self._triggered_at,
                zero_sent_after_trigger=self._zero_sent_after_trigger,
                transport_fault=self._transport_fault,
            )

    def trigger(self, reason: str) -> None:
        """锁存第一次急停原因和时间；后续触发不得覆盖事故首因。"""
        if not isinstance(reason, str):
            raise TypeError("急停原因必须是 str")
        if reason == "":
            raise ValueError("急停原因不能为空")

        with self._lock:
            if self._active:
                return
            self._active = True
            self._reason = reason
            self._triggered_at = self._clock()
            # 新一轮急停必须重新取得触发后的全零发送证据，不能沿用旧状态。
            self._zero_sent_after_trigger = False
            self._transport_fault = None

    def record_send_success(self, command: PumpCommand) -> None:
        """记录发送成功；仅急停后的完整全零命令可作为解除证据。"""
        with self._lock:
            if self._active and command == PumpCommand.safe_stop():
                self._zero_sent_after_trigger = True

    def record_transport_fault(self, reason: str) -> None:
        """在急停期间锁存当前传输故障，直至调用恢复接口。"""
        with self._lock:
            if self._active:
                self._transport_fault = reason

    def record_transport_recovered(self) -> None:
        """显式确认急停期间的传输链路已经恢复。"""
        with self._lock:
            if self._active:
                self._transport_fault = None

    def clear(self) -> None:
        """在全零已成功发送且传输正常后解除急停。"""
        with self._lock:
            if not self._active:
                return
            # 解除依据必须来自本次急停触发后的实际发送成功事件。
            if not self._zero_sent_after_trigger:
                raise RuntimeError("急停后尚无成功发送全零命令的证据")
            # 故障只能由显式恢复事件清除，不能因操作员尝试解除而被顺带抹掉。
            if self._transport_fault is not None:
                raise RuntimeError("传输正常后才能解除急停")

            self._active = False
            self._reason = None
            self._triggered_at = None
            self._zero_sent_after_trigger = False
            self._transport_fault = None
