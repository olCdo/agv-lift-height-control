"""无硬件依赖的混合闭环高度控制器。

``step`` 的安全顺序固定为：输入与时序门禁 → 锁存故障处理 → 控制状态机 →
授权门控。任何门禁失败或授权缺失都返回完整全零；本控制器永不因自动目标而下降。
"""

from __future__ import annotations

from enum import Enum
from math import isfinite

from .calibration import LIFT_PWM_LEVELS, LOWER_VALVE_LEVELS, CalibrationBundle
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


EXTERNAL_CONTROLLER_STATES = frozenset(
    {
        ControllerState.LIFT_CALIBRATION,
        ControllerState.LOWER_CALIBRATION,
        ControllerState.SURVEY,
    }
)


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
        self.fault_kind: str | None = None
        self.fault_height_mm: float | None = None
        self.fault_timestamp: float | None = None
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
        self._lift_direction_reference_mm: float | None = None
        self._lower_direction_reference_mm: float | None = None
        self._overcurrent_since: float | None = None
        self._overcurrent_pwm: int | None = None
        self._latched_overcurrent_pwm: int | None = None
        self._direction_recovery_reference_mm: float | None = None
        self._direction_recovery_since: float | None = None
        self._external_mode: ControllerState | None = None

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
        if self._external_mode is not None:
            raise RuntimeError("外部模式中禁止设置自动目标")
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
        self._record_actual_command(PumpCommand.safe_stop())
        if (
            self.state is not ControllerState.FAULT
            and self.state not in EXTERNAL_CONTROLLER_STATES
        ):
            self.state = ControllerState.MONITOR

    def set_manual_lower(self, active: bool) -> None:
        """设置人工下降意图；使能时取消自动目标，实际阀输出仍由 step 授权门控。"""
        if type(active) is not bool:
            raise TypeError("active 必须是 bool")
        if self._external_mode is not None:
            raise RuntimeError("外部模式中禁止设置人工下降")
        self._manual_lower = active
        if active:
            self._target_mm = None
            self._stable_since = None
            self._pulse_phase_started = None
            self._effective_upper_limit_mm = self.calibration.soft_upper_limit_mm
            self._record_actual_command(PumpCommand.safe_stop())
        if self.state is not ControllerState.FAULT:
            self.state = ControllerState.MANUAL_LOWER if active else ControllerState.MONITOR

    def clear_fault(self) -> None:
        """请求清故障；只有下一次 step 的所有门禁恢复后才真正退出 FAULT。"""
        self._fault_clear_requested = True
        if self.state is ControllerState.FAULT and self.fault_kind in {
            "direction",
            "lower_direction",
        }:
            self._direction_recovery_reference_mm = self.fault_height_mm
            self._direction_recovery_since = None

    def enter_external_mode(self, mode: ControllerState) -> None:
        """进入由独立标定/测量会话拥有命令源的外部安全模式。"""
        if not isinstance(mode, ControllerState) or mode not in EXTERNAL_CONTROLLER_STATES:
            raise ValueError("外部模式只能是起升标定、下降标定或上限测量")
        if self.state is ControllerState.FAULT:
            raise RuntimeError("故障未清除时禁止进入外部模式")
        self._target_mm = None
        self._manual_lower = False
        self._stable_since = None
        self._pulse_phase_started = None
        self._effective_upper_limit_mm = self.calibration.soft_upper_limit_mm
        # 命令源所有权在此原子切换；旧自动命令的方向/过流历史不能污染
        # 外部标定或 Survey 状态，否则 controller.step 会把安全模式误锁故障。
        self._record_actual_command(PumpCommand.safe_stop())
        self._last_sample = None
        self._overcurrent_since = None
        self._overcurrent_pwm = None
        self._latched_overcurrent_pwm = None
        self._fault_clear_requested = False
        self._external_mode = mode
        self.state = mode

    def exit_external_mode(self) -> None:
        """退出外部命令源模式；故障状态下调用不会绕过故障锁存。"""
        if self._external_mode is not None:
            self._external_mode = None
            self._record_actual_command(PumpCommand.safe_stop())
            if self.state is not ControllerState.FAULT:
                self.state = ControllerState.MONITOR

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
        cycle = self._begin_safety_cycle(
            now=now,
            sample=sample,
            feedback=feedback,
            lift_authorized=lift_authorized,
            lower_authorized=lower_authorized,
        )
        if cycle is None:
            return safe_stop
        timestamp, height, _checked_feedback = cycle

        if self._external_mode is not None:
            # 标定/Survey 会话是该模式唯一命令源；控制器仍执行上方门禁，
            # 但自身永远返回全零，避免与外部会话同时驱动泵。
            self._record_actual_command(safe_stop, height)
            return safe_stop

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
            self._record_actual_command(command, height)
            return command

        limit_reason = self._passive_upper_limit_reason(height)
        if limit_reason is not None:
            return self._fault(
                limit_reason,
                kind="limit",
                height_mm=height,
                timestamp=timestamp,
            )

        if self._target_mm is None:
            self.state = ControllerState.MONITOR
            self._record_actual_command(safe_stop, height)
            return safe_stop

        command = self._automatic_command(timestamp, height)
        if command.lift_pwm and not lift_authorized:
            # 未实际通电的脉冲不能继续计时，否则授权恢复时可能从等待相位开始。
            if self.state is ControllerState.TERMINAL_PULSE:
                self._pulse_phase_started = None
            command = safe_stop
        self._record_actual_command(command, height)
        return command

    def step_external(
        self,
        *,
        now: float,
        sample: HeightSample,
        feedback: PumpFeedback | None,
        desired_command: PumpCommand,
        lift_authorized: bool,
        lower_authorized: bool,
    ) -> PumpCommand:
        """仲裁标定或测量会话的期望命令，并返回唯一可下发的实际命令。"""
        if self._external_mode is None:
            raise RuntimeError("step_external 只能在外部模式中使用")
        if not isinstance(desired_command, PumpCommand):
            raise TypeError("desired_command 必须是 PumpCommand")

        safe_stop = PumpCommand.safe_stop()
        cycle = self._begin_safety_cycle(
            now=now,
            sample=sample,
            feedback=feedback,
            lift_authorized=lift_authorized,
            lower_authorized=lower_authorized,
        )
        if cycle is None:
            return safe_stop
        timestamp, height, _checked_feedback = cycle

        if desired_command.lift_pwm and desired_command.lower_valve:
            return self._fault(
                "外部命令禁止同时起升和下降",
                kind="input",
                height_mm=height,
                timestamp=timestamp,
            )
        if (desired_command.lift_pwm or desired_command.lower_valve) and not desired_command.interlock:
            return self._fault(
                "外部非零命令必须使能互锁",
                kind="input",
                height_mm=height,
                timestamp=timestamp,
            )
        if (
            self._external_mode is ControllerState.LOWER_CALIBRATION
            and desired_command.lift_pwm
        ) or (
            self._external_mode
            in {ControllerState.LIFT_CALIBRATION, ControllerState.SURVEY}
            and desired_command.lower_valve
        ):
            return self._fault(
                "外部命令方向与当前模式不一致",
                kind="input",
                height_mm=height,
                timestamp=timestamp,
            )
        plan_reason = self._external_plan_reason(desired_command)
        if plan_reason is not None:
            return self._fault(
                plan_reason,
                kind="input",
                height_mm=height,
                timestamp=timestamp,
            )

        if desired_command.lift_pwm:
            limit_reason = self._lift_upper_limit_reason(height)
            if limit_reason is not None:
                return self._fault(
                    limit_reason,
                    kind="limit",
                    height_mm=height,
                    timestamp=timestamp,
                )
            command = desired_command if lift_authorized else safe_stop
        elif desired_command.lower_valve:
            command = desired_command if lower_authorized else safe_stop
        else:
            command = safe_stop
        self._record_actual_command(command, height)
        return command

    def _begin_safety_cycle(
        self,
        *,
        now: object,
        sample: object,
        feedback: object,
        lift_authorized: object,
        lower_authorized: object,
    ) -> tuple[float, float, PumpFeedback] | None:
        """执行普通与外部命令共用的时序、反馈、方向和过流门禁。"""
        if not _finite_nonnegative(now):
            self._fault("本机 now 时间戳无效", kind="input")
            return None
        timestamp = float(now)
        if self._last_step_at is not None:
            gap = timestamp - self._last_step_at
            if gap < 0:
                self._fault("本机时钟回退", kind="input", timestamp=timestamp)
                return None
            if gap - self.config.control_loop_timeout_s > 1e-12:
                self._last_step_at = timestamp
                self._fault("控制循环已超时", kind="timeout", timestamp=timestamp)
                return None
        self._last_step_at = timestamp

        guard_reason = self._validate_inputs(
            timestamp,
            sample,
            feedback,
            lift_authorized,
            lower_authorized,
        )
        if guard_reason is not None:
            kind = (
                "direction"
                if guard_reason == "起升命令下高度方向反向"
                else "lower_direction"
                if guard_reason == "下降命令下高度方向反向"
                else "input"
            )
            fault_height = (
                float(sample.height_mm)
                if isinstance(sample, HeightSample)
                and _finite_nonnegative(sample.height_mm)
                else None
            )
            self._fault(
                guard_reason,
                kind=kind,
                height_mm=fault_height,
                timestamp=timestamp,
            )
            return None
        assert isinstance(sample, HeightSample)
        assert sample.height_mm is not None
        assert isinstance(feedback, PumpFeedback)
        height = float(sample.height_mm)

        current_reason = self._check_overcurrent(timestamp, feedback)
        if current_reason is not None:
            self._fault(
                current_reason,
                kind="overcurrent",
                height_mm=height,
                timestamp=timestamp,
            )
            return None
        if (
            self.state is ControllerState.FAULT
            and self._fault_clear_requested
            and self._latched_overcurrent_pwm is not None
        ):
            peak = self.calibration.peak_current_by_pwm[self._latched_overcurrent_pwm]
            if feedback.current_raw > self.config.current_multiplier * peak:
                self._fault("泵电流仍高于标定阈值，过流条件未恢复")
                return None

        self._last_sample = sample
        if self.state is ControllerState.FAULT and self.fault_kind in {
            "direction",
            "lower_direction",
        }:
            self._handle_direction_fault_recovery(timestamp, height)
            return None
        if self.state is ControllerState.FAULT:
            self._record_actual_command(PumpCommand.safe_stop(), height)
            if not self._fault_clear_requested:
                return None
            cleared_kind = self.fault_kind
            self._clear_latched_fault()
            if cleared_kind == "limit":
                # 限位故障切换到人工下降时，清除周期仍保持全零。
                return None
        return timestamp, height, feedback

    def _record_actual_command(
        self, command: PumpCommand, height_mm: float | None = None
    ) -> None:
        """只按实际下发命令维护连续起升或下降段的累计方向基准。"""
        self._last_command = command
        if command.lift_pwm > 0:
            self._lower_direction_reference_mm = None
            if height_mm is None:
                return
            self._lift_direction_reference_mm = (
                height_mm
                if self._lift_direction_reference_mm is None
                else max(self._lift_direction_reference_mm, height_mm)
            )
        elif command.lower_valve > 0:
            self._lift_direction_reference_mm = None
            if height_mm is None:
                return
            self._lower_direction_reference_mm = (
                height_mm
                if self._lower_direction_reference_mm is None
                else min(self._lower_direction_reference_mm, height_mm)
            )
        else:
            self._lift_direction_reference_mm = None
            self._lower_direction_reference_mm = None

    def _external_plan_reason(self, command: PumpCommand) -> str | None:
        """外部接口只接受当前会话会实际生成的离散实测档位。"""
        if command.accel != 0 or command.decel != 0:
            return "外部实测计划命令的加减速字段必须为零"
        if (
            self._external_mode is ControllerState.LIFT_CALIBRATION
            and command.lift_pwm
            and command.lift_pwm not in LIFT_PWM_LEVELS
        ):
            return "起升标定命令不属于离散实测计划"
        if (
            self._external_mode is ControllerState.SURVEY
            and command.lift_pwm
            and command.lift_pwm != self.calibration.min_stable_pwm
        ):
            return "上限测量命令不属于离散实测计划"
        if (
            self._external_mode is ControllerState.LOWER_CALIBRATION
            and command.lower_valve
            and command.lower_valve not in LOWER_VALVE_LEVELS
        ):
            return "下降标定命令不属于离散实测计划"
        return None

    def _passive_upper_limit_reason(self, height_mm: float) -> str | None:
        """非下降状态越界即锁存；人工下降恢复不调用本门禁。"""
        if height_mm > self.config.absolute_max_height_mm:
            return "当前高度超过绝对上限"
        limit = self._effective_upper_limit_mm
        if limit is not None and height_mm > limit:
            return "当前高度已越过有效软限位"
        return None

    def _lift_upper_limit_reason(self, height_mm: float) -> str | None:
        """任何实际起升意图在到达软/绝对上限时都必须失败关闭。"""
        if height_mm >= self.config.absolute_max_height_mm:
            return "达到绝对上限后仍请求起升"
        limit = self._effective_upper_limit_mm
        if limit is not None and height_mm >= limit:
            return "达到软限位后仍请求起升"
        return None

    def _clear_latched_fault(self) -> None:
        """门禁恢复且已明确请求后，清除通用锁存故障元数据。"""
        self.state = (
            self._external_mode
            if self._external_mode is not None
            else ControllerState.MANUAL_LOWER
            if self._manual_lower
            else ControllerState.IDLE
            if self._target_mm is not None
            else ControllerState.MONITOR
        )
        self.fault_reason = None
        self.fault_kind = None
        self.fault_height_mm = None
        self.fault_timestamp = None
        self.trial_failed = False
        self._fault_clear_requested = False
        self._overcurrent_since = None
        self._overcurrent_pwm = None
        self._latched_overcurrent_pwm = None

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
            and self._lift_direction_reference_mm is not None
            and height
            < self._lift_direction_reference_mm - self.config.direction_tolerance_mm
        ):
            return "起升命令下高度方向反向"
        if (
            self._last_command.lower_valve > 0
            and self._lower_direction_reference_mm is not None
            and height
            > self._lower_direction_reference_mm + self.config.direction_tolerance_mm
        ):
            return "下降命令下高度方向反向"
        return None

    def _check_overcurrent(self, now: float, feedback: PumpFeedback) -> str | None:
        pwm = self._last_command.lift_pwm
        peak = self.calibration.peak_current_by_pwm.get(pwm)
        if pwm == 0 or peak is None:
            self._overcurrent_since = None
            self._overcurrent_pwm = None
            return None
        threshold = self.config.current_multiplier * peak
        if self._overcurrent_pwm != pwm:
            # 过流持续时间只属于具体实测 PWM；换挡后必须从新 PWM 的首帧
            # 重新计时，不能沿用上一档积累的时间。
            self._overcurrent_pwm = pwm
            self._overcurrent_since = now if feedback.current_raw > threshold else None
            return None
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
            return self._fault(
                f"目标超调 {overshoot:.3f} mm，超过安全上限",
                kind="overshoot",
                height_mm=height_mm,
                timestamp=now,
            )
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
            raw_pwm = (
                self.calibration.min_stable_pwm
                + scale
                * (self.calibration.coarse_pwm - self.calibration.min_stable_pwm)
            )
            # 向上量化到已标定的 5-PWM 档，牺牲少量平滑性换取每条实际命令
            # 都有同 PWM 峰值电流可用于持续过流保护。
            measured_levels = (
                level
                for level in LIFT_PWM_LEVELS
                if self.calibration.min_stable_pwm
                <= level
                <= self.calibration.coarse_pwm
                and level >= raw_pwm
            )
            pwm = next(measured_levels, self.calibration.coarse_pwm)
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
        limit_reason = self._lift_upper_limit_reason(height_mm)
        if limit_reason is not None:
            return self._fault(limit_reason, kind="limit", height_mm=height_mm)
        return PumpCommand(interlock=True, lift_pwm=pwm)

    def _handle_direction_fault_recovery(
        self, now: float, height_mm: float
    ) -> PumpCommand:
        """方向故障仅在明确请求后、连续稳定观察完成的下一周期才可运动。"""
        safe_stop = PumpCommand.safe_stop()
        self._record_actual_command(safe_stop, height_mm)
        if not self._fault_clear_requested:
            return safe_stop
        reference = self._direction_recovery_reference_mm
        if reference is None:
            reference = height_mm
            self._direction_recovery_reference_mm = height_mm
        reverse_continues = (
            height_mm > reference + self.config.direction_tolerance_mm
            if self.fault_kind == "lower_direction"
            else height_mm < reference - self.config.direction_tolerance_mm
        )
        if reverse_continues:
            # 反向趋势继续超过容差时，以新的极值重启稳定观察窗口。
            self._direction_recovery_reference_mm = height_mm
            self._direction_recovery_since = now
            return safe_stop
        if self._direction_recovery_since is None:
            self._direction_recovery_since = now
            return safe_stop
        if now - self._direction_recovery_since + 1e-12 < self.config.stable_time_s:
            return safe_stop

        self._clear_latched_fault()
        self._direction_recovery_reference_mm = None
        self._direction_recovery_since = None
        # 清除完成周期仍返回零；下一周期才重新进入自动状态机。
        return safe_stop

    def _fault(
        self,
        reason: str,
        *,
        kind: str | None = None,
        height_mm: float | None = None,
        timestamp: float | None = None,
    ) -> PumpCommand:
        if self.state is not ControllerState.FAULT:
            self.fault_kind = kind or "safety"
            self.fault_height_mm = height_mm
            self.fault_timestamp = timestamp
            if self.fault_kind in {"direction", "lower_direction"}:
                self._direction_recovery_reference_mm = height_mm
                self._direction_recovery_since = None
        self.state = ControllerState.FAULT
        self.fault_reason = reason
        self.trial_failed = True
        self._fault_clear_requested = False
        self._stable_since = None
        self._pulse_phase_started = None
        self._overcurrent_since = None
        self._record_actual_command(PumpCommand.safe_stop())
        return self._last_command

    def _validate_height_argument(self, name: str, value: object) -> float:
        if not _finite_nonnegative(value) or value <= 0:
            raise ValueError(f"{name} 必须是有限正数")
        result = float(value)
        if result > self.config.absolute_max_height_mm:
            raise ValueError(f"{name} 不得超过 2900 mm 绝对上限")
        return result
