"""无硬件依赖的混合闭环高度控制器。

``step`` 的安全顺序固定为：输入与时序门禁 → 锁存故障处理 → 控制状态机 →
授权门控。任何门禁失败或授权缺失都返回完整全零；本控制器永不因自动目标而下降。
"""

from __future__ import annotations

from enum import Enum
from math import isfinite

from .calibration import CalibrationBundle
from .config import ControlConfig
from .types import HeightSample, PumpCommand, PumpFeedback


class ControllerState(str, Enum):
    MONITOR = "monitor"
    LIFT_CALIBRATION = "lift_calibration"
    LOWER_CALIBRATION = "lower_calibration"
    IDLE = "idle"
    COARSE_LIFT = "coarse_lift"
    P_CONTROL = "p_control"
    TERMINAL_PULSE = "terminal_pulse"
    HOLD = "hold"
    MANUAL_LOWER = "manual_lower"
    SURVEY = "survey"
    FAULT = "fault"


def _finite_nonnegative(value: object) -> bool:
    return type(value) in {int, float} and isfinite(float(value)) and value >= 0


class HeightController:
    """根据标定摘要生成泵命令并锁存所有控制层故障。"""

    def __init__(
        self,
        config: ControlConfig,
        calibration: CalibrationBundle,
        *,
        feedback_timeout_s: float = 0.15,
    ) -> None:
        if not isinstance(config, ControlConfig):
            raise TypeError("config 必须是 ControlConfig")
        if not isinstance(calibration, CalibrationBundle):
            raise TypeError("calibration 必须是 CalibrationBundle")
        if (
            not _finite_nonnegative(feedback_timeout_s)
            or feedback_timeout_s <= 0
            or feedback_timeout_s > 0.15
        ):
            raise ValueError("feedback_timeout_s 必须是 0..0.15 秒内的有限正数")
        self.config = config
        self.calibration = calibration
        self.feedback_timeout_s = float(feedback_timeout_s)

        self.slow_zone_mm = max(50.0, 5.0 * calibration.max_coast_mm)
        self.pulse_zone_mm = max(10.0, 3.0 * calibration.max_coast_mm)
        # 极端或未来 schema 数据退化时仍保留非空 P 区，禁止除零和区间反转。
        if self.slow_zone_mm <= self.pulse_zone_mm:
            self.slow_zone_mm = self.pulse_zone_mm + 1.0
        self.pulse_on_s = min(max(calibration.response_delay_s, 0.1), 0.3)
        self.pulse_wait_s = min(
            max(calibration.response_delay_s + config.stable_time_s, 0.3), 1.0
        )

        self.state = ControllerState.MONITOR
        self.fault_reason: str | None = None
        self.trial_failed = False
        self._target_mm: float | None = None
        self._effective_upper_limit_mm = calibration.soft_upper_limit_mm
        self._manual_lower = False
        self._fault_clear_requested = False
        self._stable_since: float | None = None
        self._pulse_phase_started: float | None = None
        self._last_step_at: float | None = None
        self._last_sample: HeightSample | None = None
        self._last_command = PumpCommand.safe_stop()
        self._overcurrent_since: float | None = None
        self._latched_overcurrent_pwm: int | None = None

    @property
    def target_mm(self) -> float | None:
        return self._target_mm

    @property
    def effective_upper_limit_mm(self) -> float | None:
        return self._effective_upper_limit_mm

    def set_target(
        self,
        target_mm: float,
        *,
        temporary_max_height_mm: float | None = None,
    ) -> None:
        """设置自动起升目标；无持久软限位时强制提供人工临时上限。"""
        target = self._validate_height_argument("target_mm", target_mm)
        persistent = self.calibration.soft_upper_limit_mm
        if temporary_max_height_mm is None:
            if persistent is None:
                raise ValueError("无持久软限位时必须提供人工临时最大高度")
            effective = persistent
        else:
            temporary = self._validate_height_argument(
                "temporary_max_height_mm", temporary_max_height_mm
            )
            if temporary > self.config.absolute_max_height_mm:
                raise ValueError("临时最大高度不得超过绝对上限 2900 mm")
            effective = temporary if persistent is None else min(temporary, persistent)
        if target > effective:
            raise ValueError(f"目标高度不得超过有效软限位 {effective:g} mm")
        self._target_mm = target
        self._effective_upper_limit_mm = effective
        self._manual_lower = False
        self._stable_since = None
        self._pulse_phase_started = None
        self.trial_failed = False
        if self.state is not ControllerState.FAULT:
            self.state = ControllerState.IDLE

    def cancel(self) -> None:
        """取消自动和手动请求，但不清除已锁存故障。"""
        self._target_mm = None
        self._manual_lower = False
        self._stable_since = None
        self._pulse_phase_started = None
        self._effective_upper_limit_mm = self.calibration.soft_upper_limit_mm
        if self.state is not ControllerState.FAULT:
            self.state = ControllerState.MONITOR

    def set_manual_lower(self, active: bool) -> None:
        """设置人工下降意图；使能时取消自动目标，实际阀输出仍由 step 授权门控。"""
        if type(active) is not bool:
            raise TypeError("active 必须是 bool")
        self._manual_lower = active
        if active:
            self._target_mm = None
            self._stable_since = None
            self._pulse_phase_started = None
            self._effective_upper_limit_mm = self.calibration.soft_upper_limit_mm
        if self.state is not ControllerState.FAULT:
            self.state = ControllerState.MANUAL_LOWER if active else ControllerState.MONITOR

    def clear_fault(self) -> None:
        """请求清故障；只有下一次 step 的所有门禁恢复后才真正退出 FAULT。"""
        self._fault_clear_requested = True

    def step(
        self,
        *,
        now: float,
        sample: HeightSample,
        feedback: PumpFeedback | None,
        lift_authorized: bool,
        lower_authorized: bool,
    ) -> PumpCommand:
        """执行一个控制周期，并返回经过全部失效保护后的实际命令。"""
        safe_stop = PumpCommand.safe_stop()
        if not _finite_nonnegative(now):
            return self._fault("本机 now 时间戳无效")
        timestamp = float(now)

        # 只要本机时钟有效就记录控制循环到达，其他输入故障不应伪装成线程停顿。
        if self._last_step_at is not None:
            gap = timestamp - self._last_step_at
            if gap < 0:
                return self._fault("本机时钟回退")
            if gap - self.config.control_loop_timeout_s > 1e-12:
                self._last_step_at = timestamp
                return self._fault("控制循环已超时")
        self._last_step_at = timestamp

        guard_reason = self._validate_inputs(
            timestamp,
            sample,
            feedback,
            lift_authorized,
            lower_authorized,
        )
        if guard_reason is not None:
            return self._fault(guard_reason)
        assert feedback is not None
        assert sample.height_mm is not None

        current_reason = self._check_overcurrent(timestamp, feedback)
        if current_reason is not None:
            return self._fault(current_reason)

        if (
            self.state is ControllerState.FAULT
            and self._fault_clear_requested
            and self._latched_overcurrent_pwm is not None
        ):
            peak = self.calibration.peak_current_by_pwm[self._latched_overcurrent_pwm]
            if feedback.current_raw > self.config.current_multiplier * peak:
                return self._fault("泵电流仍高于标定阈值，过流条件未恢复")

        # 门禁通过后才更新相邻样本基准；用于下一周期速度和起升方向检查。
        self._last_sample = sample
        if self.state is ControllerState.FAULT:
            self._last_command = safe_stop
            if not self._fault_clear_requested:
                return safe_stop
            self.state = (
                ControllerState.MANUAL_LOWER
                if self._manual_lower
                else ControllerState.IDLE
                if self._target_mm is not None
                else ControllerState.MONITOR
            )
            self.fault_reason = None
            self.trial_failed = False
            self._fault_clear_requested = False
            self._overcurrent_since = None
            self._latched_overcurrent_pwm = None

        if self._manual_lower:
            self.state = ControllerState.MANUAL_LOWER
            command = (
                PumpCommand(
                    interlock=True,
                    lower_valve=self.calibration.lower_comfortable_valve,
                )
                if lower_authorized
                else safe_stop
            )
            self._last_command = command
            return command

        if self._target_mm is None:
            self.state = ControllerState.MONITOR
            self._last_command = safe_stop
            return safe_stop

        command = self._automatic_command(timestamp, float(sample.height_mm))
        if command.lift_pwm and not lift_authorized:
            # 未实际通电的脉冲不能继续计时，否则授权恢复时可能从等待相位开始。
            if self.state is ControllerState.TERMINAL_PULSE:
                self._pulse_phase_started = None
            command = safe_stop
        self._last_command = command
        return command

    def _validate_inputs(
        self,
        now: float,
        sample: object,
        feedback: object,
        lift_authorized: object,
        lower_authorized: object,
    ) -> str | None:
        if type(lift_authorized) is not bool or type(lower_authorized) is not bool:
            return "授权输入必须是 bool"
        if (
            not isinstance(sample, HeightSample)
            or type(sample.valid) is not bool
            or not sample.valid
        ):
            return "高度样本无效"
        if not _finite_nonnegative(sample.timestamp):
            return "传感器时间戳无效"
        sample_age = now - float(sample.timestamp)
        if sample_age < 0 or sample_age - self.config.sensor_timeout_s > 1e-12:
            return "传感器样本已超时或来自未来"
        if type(sample.raw_count) is not int or not 0 <= sample.raw_count <= 0xFFFFFFFF:
            return "高度样本 raw_count 不合理"
        if not _finite_nonnegative(sample.height_mm):
            return "当前高度不合理"
        height = float(sample.height_mm)
        if height > self.config.absolute_max_height_mm:
            return "当前高度超过绝对上限"

        if not isinstance(feedback, PumpFeedback):
            return "CAN 泵反馈不存在"
        if not _finite_nonnegative(feedback.timestamp):
            return "CAN 反馈时间戳无效"
        feedback_age = now - float(feedback.timestamp)
        if feedback_age < 0 or feedback_age - self.feedback_timeout_s > 1e-12:
            return "CAN 泵反馈已超时或来自未来"
        if type(feedback.fault_code) is not int or feedback.fault_code != 0:
            return f"CAN 泵反馈故障码 {feedback.fault_code}"
        if type(feedback.current_raw) is not int or not 0 <= feedback.current_raw <= 65535:
            return "CAN 泵电流反馈不合理"

        active_limit = self._effective_upper_limit_mm
        if active_limit is not None and height > active_limit:
            return "当前高度已越过有效软限位"

        if self._last_sample is not None and self._last_sample.height_mm is not None:
            delta_t = float(sample.timestamp) - float(self._last_sample.timestamp)
            delta_height = height - float(self._last_sample.height_mm)
            if delta_t < 0:
                return "传感器时间戳回退"
            if delta_t == 0 and delta_height != 0:
                return "相同时间戳出现不同高度"
            if delta_t > 0 and abs(delta_height) / delta_t > self.config.max_speed_mm_s:
                return "相邻高度样本速度超过上限"
            if (
                self._last_command.lift_pwm > 0
                and delta_height < -self.config.direction_tolerance_mm
            ):
                return "起升命令下高度方向反向"
        return None

    def _check_overcurrent(self, now: float, feedback: PumpFeedback) -> str | None:
        pwm = self._last_command.lift_pwm
        peak = self.calibration.peak_current_by_pwm.get(pwm)
        if pwm == 0 or peak is None:
            self._overcurrent_since = None
            return None
        threshold = self.config.current_multiplier * peak
        if feedback.current_raw <= threshold:
            self._overcurrent_since = None
            return None
        if self._overcurrent_since is None:
            self._overcurrent_since = now
            return None
        if now - self._overcurrent_since + 1e-12 >= self.config.current_duration_s:
            self._latched_overcurrent_pwm = pwm
            return f"PWM {pwm} 泵电流连续超限"
        return None

    def _automatic_command(self, now: float, height_mm: float) -> PumpCommand:
        assert self._target_mm is not None
        error = self._target_mm - height_mm
        overshoot = -error
        if overshoot > self.config.overshoot_limit_mm:
            return self._fault(f"目标超调 {overshoot:.3f} mm，超过安全上限")
        if error < -self.config.tolerance_mm:
            self.state = ControllerState.IDLE
            self.trial_failed = True
            self._stable_since = None
            self._pulse_phase_started = None
            return PumpCommand.safe_stop()
        if abs(error) <= self.config.tolerance_mm:
            self._pulse_phase_started = None
            if self._stable_since is None:
                self._stable_since = now
            if now - self._stable_since + 1e-12 >= self.config.stable_time_s:
                self.state = ControllerState.HOLD
            else:
                self.state = ControllerState.IDLE
            return PumpCommand.safe_stop()

        self._stable_since = None
        if error > self.slow_zone_mm:
            self.state = ControllerState.COARSE_LIFT
            self._pulse_phase_started = None
            return self._lift_command(self.calibration.coarse_pwm, height_mm)
        if error > self.pulse_zone_mm:
            self.state = ControllerState.P_CONTROL
            self._pulse_phase_started = None
            scale = (error - self.pulse_zone_mm) / (
                self.slow_zone_mm - self.pulse_zone_mm
            )
            pwm = round(
                self.calibration.min_stable_pwm
                + scale
                * (self.calibration.coarse_pwm - self.calibration.min_stable_pwm)
            )
            pwm = min(max(pwm, self.calibration.min_stable_pwm), self.calibration.coarse_pwm)
            return self._lift_command(pwm, height_mm)

        if self.state is not ControllerState.TERMINAL_PULSE or self._pulse_phase_started is None:
            self._pulse_phase_started = now
        self.state = ControllerState.TERMINAL_PULSE
        elapsed = now - self._pulse_phase_started
        if elapsed + 1e-12 < self.pulse_on_s:
            return self._lift_command(self.calibration.min_stable_pwm, height_mm)
        if elapsed + 1e-12 < self.pulse_on_s + self.pulse_wait_s:
            return PumpCommand.safe_stop()
        self._pulse_phase_started = now
        return self._lift_command(self.calibration.min_stable_pwm, height_mm)

    def _lift_command(self, pwm: int, height_mm: float) -> PumpCommand:
        limit = self._effective_upper_limit_mm
        if limit is not None and height_mm >= limit:
            return self._fault("达到软限位后仍请求起升")
        if height_mm >= self.config.absolute_max_height_mm:
            return self._fault("达到绝对上限后仍请求起升")
        return PumpCommand(interlock=True, lift_pwm=pwm)

    def _fault(self, reason: str) -> PumpCommand:
        self.state = ControllerState.FAULT
        self.fault_reason = reason
        self.trial_failed = True
        self._fault_clear_requested = False
        self._stable_since = None
        self._pulse_phase_started = None
        self._overcurrent_since = None
        self._last_command = PumpCommand.safe_stop()
        return self._last_command

    def _validate_height_argument(self, name: str, value: object) -> float:
        if not _finite_nonnegative(value) or value <= 0:
            raise ValueError(f"{name} 必须是有限正数")
        result = float(value)
        if result > self.config.absolute_max_height_mm:
            raise ValueError(f"{name} 不得超过 2900 mm 绝对上限")
        return result
