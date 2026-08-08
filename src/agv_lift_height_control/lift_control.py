"""面向业务入口的升降高度公共控制门面。"""

from __future__ import annotations

from collections.abc import Callable
from math import isfinite
from numbers import Real
from threading import RLock
from time import monotonic

from .controller import HeightController
from .emergency_stop import EmergencyStopLatch
from .types import HeightSample, PumpCommand, PumpFeedback


class LiftHeightControl:
    """串行化目标、控制周期与锁存急停状态转换。"""

    def __init__(
        self,
        controller: HeightController,
        emergency_stop_latch: EmergencyStopLatch,
        *,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        self.controller = controller
        self.emergency_stop_latch = emergency_stop_latch
        self._clock = clock
        self._lock = RLock()
        self._last_sample: HeightSample | None = None
        self._last_feedback: PumpFeedback | None = None

    def set_target_height(
        self,
        target_mm: float,
        temporary_max_height_mm: float | None = None,
    ) -> None:
        """设置自主定高目标；锁存急停期间不得形成新的运动意图。"""
        with self._lock:
            if self.emergency_stop_latch.snapshot().active:
                raise RuntimeError("急停锁存期间禁止设置目标高度")
            self.controller.set_target(
                target_mm,
                temporary_max_height_mm=temporary_max_height_mm,
            )

    def emergency_stop(self, reason: str) -> None:
        """先关闭最底层发送门禁，再串行撤销控制器内的全部运动意图。"""
        # 先 trigger 共享锁存，保证本函数尚未取得门面锁时，CanPump 已经只能发送全零；
        # 若反过来先改控制器，另一个线程可能在底层门禁尚未生效的窗口发出旧非零命令。
        self.emergency_stop_latch.trigger(reason)
        with self._lock:
            emergency = self.emergency_stop_latch.snapshot()
            if not emergency.active:
                # 解除线程可能在 trigger 与取得门面锁之间完成上一轮解除；此时必须
                # 重新锁存为新一轮急停，不能留下“控制器急停、底层门禁已放行”。
                self.emergency_stop_latch.trigger(reason)
                emergency = self.emergency_stop_latch.snapshot()
            assert emergency.reason is not None
            self.controller.enter_emergency_stop(emergency.reason)

    def update(
        self,
        now: float,
        sample: HeightSample | None,
        feedback: PumpFeedback | None,
    ) -> PumpCommand:
        """缓存最新观测并执行自主控制周期，自动升降不依赖键盘死手授权。"""
        with self._lock:
            self._last_sample = sample
            self._last_feedback = feedback
            emergency = self.emergency_stop_latch.snapshot()
            if emergency.active:
                # CanPump 可能从发送线程先触发急停；控制器必须在本周期同步同一首因，
                # 但仍调用 step，以维持公共更新入口始终返回控制器最终安全命令。
                assert emergency.reason is not None
                self.controller.enter_emergency_stop(emergency.reason)
            return self.controller.step(
                now=now,
                sample=sample,
                feedback=feedback,
                lift_authorized=True,
                lower_authorized=True,
            )

    def clear_emergency_stop(self) -> None:
        """仅在新鲜、健康观测和底层全零证据齐备时原子解除急停。"""
        with self._lock:
            emergency = self.emergency_stop_latch.snapshot()
            if not emergency.active:
                return
            assert emergency.reason is not None
            # 急停也可能由泵侧先锁存；在解除前先同步控制器，确保旧目标和方向状态
            # 已撤销。后续健康校验失败只会保持更严格的急停状态，不会形成部分解除。
            self.controller.enter_emergency_stop(emergency.reason)

            # 所有可预检条件必须先于 latch.clear；否则后续校验失败会造成底层已放行、
            # 控制器仍停在 EMERGENCY_STOP 的部分解除状态。
            now = self._validated_now()
            sample = self._last_sample
            feedback = self._last_feedback
            if not isinstance(sample, HeightSample):
                raise RuntimeError("缺少最近高度样本，禁止解除急停")
            if not isinstance(feedback, PumpFeedback):
                raise RuntimeError("缺少最近 CAN 泵反馈，禁止解除急停")
            if type(sample.valid) is not bool or not sample.valid:
                raise RuntimeError("最近高度样本无效，禁止解除急停")
            if type(sample.height_mm) not in {int, float} or not isfinite(
                float(sample.height_mm)
            ):
                raise RuntimeError("最近高度样本缺少有效高度，禁止解除急停")
            self._validate_age(
                now,
                sample.timestamp,
                self.controller.config.sensor_timeout_s,
                "高度样本",
            )
            self._validate_age(
                now,
                feedback.timestamp,
                self.controller.feedback_timeout_s,
                "CAN 泵反馈",
            )
            if type(feedback.fault_code) is not int or feedback.fault_code != 0:
                raise RuntimeError(
                    f"CAN 泵反馈故障码 {feedback.fault_code} 未清零，禁止解除急停"
                )

            # latch.clear 还会原子核验本次急停后的全零成功发送证据及传输恢复状态。
            # guard 与 trigger、发送门禁共用底层锁，使“锁存解除 + 控制器退出”成为
            # 一个复合状态转换；新一轮急停只能在两者全部完成后开始。
            with self.emergency_stop_latch.state_transition_guard():
                current = self.emergency_stop_latch.snapshot()
                if (
                    not current.active
                    or current.reason != emergency.reason
                    or current.triggered_at != emergency.triggered_at
                ):
                    raise RuntimeError("急停锁存状态已变化，禁止继续解除")
                self.emergency_stop_latch.clear()
                self.controller.exit_emergency_stop()

    def _validated_now(self) -> float:
        value = self._clock()
        if isinstance(value, bool) or not isinstance(value, Real):
            raise RuntimeError("解除急停时钟必须返回实数")
        now = float(value)
        if not isfinite(now) or now < 0:
            raise RuntimeError("解除急停时钟必须是有限非负时间")
        return now

    @staticmethod
    def _validate_age(now: float, timestamp: object, timeout_s: float, label: str) -> None:
        if isinstance(timestamp, bool) or not isinstance(timestamp, Real):
            raise RuntimeError(f"最近{label}时间戳无效，禁止解除急停")
        value = float(timestamp)
        if not isfinite(value) or value < 0:
            raise RuntimeError(f"最近{label}时间戳无效，禁止解除急停")
        age = now - value
        if age < 0:
            raise RuntimeError(f"最近{label}来自未来，禁止解除急停")
        if age - timeout_s > 1e-12:
            raise RuntimeError(f"最近{label}已超时，禁止解除急停")
