"""无真实时间等待、无硬件依赖的确定性液压升降仿真。"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from math import exp, isfinite

from .types import HeightSample, PumpCommand, PumpFeedback


@dataclass(frozen=True)
class HydraulicSnapshot:
    """同一仿真时刻的高度样本与 CAN 泵反馈。"""

    now: float
    height_mm: float
    velocity_mm_s: float
    sample: HeightSample
    feedback: PumpFeedback


@dataclass(frozen=True)
class _VelocitySegment:
    """一段待响应延迟后执行的目标速度历史。"""

    start_s: float
    end_s: float
    target_velocity_mm_s: float | None


def _number(name: str, value: object, *, minimum: float, strict: bool = False) -> float:
    if type(value) not in {int, float} or not isfinite(float(value)):
        raise ValueError(f"{name} 必须是有限数字")
    result = float(value)
    if result < minimum or (strict and result == minimum):
        relation = "大于" if strict else "不小于"
        raise ValueError(f"{name} 必须{relation} {minimum}")
    return result


class HydraulicLiftSimulator:
    """模拟 PWM 死区、响应延迟、速度以及停泵后的指数滑行衰减。

    默认使用内部固定步长时钟。测试也可令 ``fixed_step_s=None`` 并注入一个手工
    推进的 ``clock``；两种模式都不会调用 ``sleep``。
    """

    def __init__(
        self,
        *,
        initial_height_mm: float = 0.0,
        min_lift_pwm: int = 45,
        response_delay_s: float = 0.1,
        max_lift_speed_mm_s: float = 300.0,
        max_lower_speed_mm_s: float = 180.0,
        coast_decay_s: float = 0.1,
        fixed_step_s: float | None = 0.05,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.height_mm = _number("initial_height_mm", initial_height_mm, minimum=0)
        if type(min_lift_pwm) is not int or not 1 <= min_lift_pwm <= 99:
            raise ValueError("min_lift_pwm 必须是 1..99 的整数")
        self.min_lift_pwm = min_lift_pwm
        self.response_delay_s = _number(
            "response_delay_s", response_delay_s, minimum=0
        )
        self.max_lift_speed_mm_s = _number(
            "max_lift_speed_mm_s", max_lift_speed_mm_s, minimum=0, strict=True
        )
        self.max_lower_speed_mm_s = _number(
            "max_lower_speed_mm_s", max_lower_speed_mm_s, minimum=0, strict=True
        )
        self.coast_decay_s = _number(
            "coast_decay_s", coast_decay_s, minimum=0, strict=True
        )
        if fixed_step_s is None and clock is None:
            raise ValueError("非固定步长模式必须注入 clock")
        self.fixed_step_s = (
            None
            if fixed_step_s is None
            else _number("fixed_step_s", fixed_step_s, minimum=0, strict=True)
        )
        self._clock = clock
        self._now = _number("clock", clock() if clock is not None else 0.0, minimum=0)
        self._velocity_mm_s = 0.0
        self._velocity_history: deque[_VelocitySegment] = deque()
        self._current_raw = 0

    @property
    def now(self) -> float:
        return self._now

    @property
    def velocity_mm_s(self) -> float:
        return self._velocity_mm_s

    def observe(self) -> HydraulicSnapshot:
        """读取当前快照，不推进仿真时钟。"""
        raw_count = min(max(round(self.height_mm * 1000.0), 0), 0xFFFFFFFF)
        return HydraulicSnapshot(
            now=self._now,
            height_mm=self.height_mm,
            velocity_mm_s=self._velocity_mm_s,
            sample=HeightSample(self._now, raw_count, self.height_mm, True, None),
            feedback=PumpFeedback(self._now, self._current_raw, 0, 0),
        )

    def advance(
        self, command: PumpCommand, *, dt_s: float | None = None
    ) -> HydraulicSnapshot:
        """按给定或固定步长推进一次；禁止模拟同时升降的非法命令。"""
        if not isinstance(command, PumpCommand):
            raise TypeError("command 必须是 PumpCommand")
        if command.lift_pwm > 0 and command.lower_valve > 0:
            raise ValueError("仿真拒绝同时起升和下降")
        duration, next_now = self._resolve_step(dt_s)

        target_velocity = self._command_target_velocity(command)
        self._velocity_history.append(
            _VelocitySegment(self._now, next_now, target_velocity)
        )
        # 响应延迟是命令到机械动作的时间平移，不会因当前帧已经归零而删除
        # 之前的液压脉冲；这与现场“100 ms停泵后才开始位移”的日志一致。
        self._integrate_delayed_history(
            self._now - self.response_delay_s,
            next_now - self.response_delay_s,
        )

        if command.interlock and command.lift_pwm > self.min_lift_pwm:
            self._current_raw = command.lift_pwm * 10
        elif command.interlock and command.lower_valve > 0:
            self._current_raw = 0
        else:
            self._current_raw = 0

        self.height_mm = max(0.0, self.height_mm)
        self._now = next_now
        return self.observe()

    def _resolve_step(self, dt_s: float | None) -> tuple[float, float]:
        if dt_s is not None:
            duration = _number("dt_s", dt_s, minimum=0, strict=True)
            return duration, self._now + duration
        if self.fixed_step_s is not None:
            return self.fixed_step_s, self._now + self.fixed_step_s
        assert self._clock is not None
        next_now = _number("clock", self._clock(), minimum=0)
        duration = next_now - self._now
        if duration <= 0:
            raise ValueError("注入时钟必须单调前进")
        return duration, next_now

    def _command_target_velocity(self, command: PumpCommand) -> float | None:
        if command.interlock and command.lift_pwm > self.min_lift_pwm:
            return self.max_lift_speed_mm_s * (
                (command.lift_pwm - self.min_lift_pwm) / (100 - self.min_lift_pwm)
            )
        if command.interlock and command.lower_valve > 0:
            return -self.max_lower_speed_mm_s * (command.lower_valve / 255.0)
        return None

    def _integrate_delayed_history(self, source_start: float, source_end: float) -> None:
        """积分延迟前的命令区间；无命令的间隙继续按当前速度指数滑行。"""
        cursor = source_start
        for segment in self._velocity_history:
            if segment.end_s <= cursor:
                continue
            if segment.start_s >= source_end:
                break
            if cursor < segment.start_s:
                gap_end = min(segment.start_s, source_end)
                self._integrate_coast(gap_end - cursor)
                cursor = gap_end
            overlap_start = max(cursor, segment.start_s)
            overlap_end = min(source_end, segment.end_s)
            if overlap_end <= overlap_start:
                continue
            segment_duration = overlap_end - overlap_start
            if segment.target_velocity_mm_s is None:
                self._integrate_coast(segment_duration)
            else:
                self._velocity_mm_s = segment.target_velocity_mm_s
                self.height_mm += self._velocity_mm_s * segment_duration
            cursor = overlap_end
        if cursor < source_end:
            self._integrate_coast(source_end - cursor)

        while (
            self._velocity_history
            and self._velocity_history[0].end_s <= source_start
        ):
            self._velocity_history.popleft()

    def _integrate_coast(self, duration: float) -> None:
        if duration <= 0 or self._velocity_mm_s == 0:
            return
        decay = exp(-duration / self.coast_decay_s)
        self.height_mm += (
            self._velocity_mm_s * self.coast_decay_s * (1.0 - decay)
        )
        self._velocity_mm_s *= decay
        if abs(self._velocity_mm_s) < 1e-9:
            self._velocity_mm_s = 0.0
