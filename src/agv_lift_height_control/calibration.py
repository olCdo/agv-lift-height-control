"""升降标定、软上限测量和严格持久化模型。

本模块只生成 :class:`PumpCommand`，不连接 CAN、不写传感器零点。标定调用方必须
在每次 ``step`` 传入实时授权；授权失效会丢弃当前未完成试验并立即返回全零。
"""

from __future__ import annotations

import json
import os
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass, replace
from math import isfinite
from pathlib import Path
from types import MappingProxyType
from typing import Any

from .config import ControlConfig
from .types import HeightSample, PumpCommand, PumpFeedback

LIFT_PWM_LEVELS = tuple(range(40, 81, 5))
LOWER_VALVE_LEVELS = tuple(range(0x10, 0xA1, 0x10))
CALIBRATION_SCHEMA_VERSION = 1


class CalibrationError(ValueError):
    """标定计划、测量值或持久化数据不满足安全约束。"""


def _finite_number(name: str, value: object, *, minimum: float | None = None) -> float:
    if type(value) not in {int, float} or not isfinite(float(value)):
        raise CalibrationError(f"{name} 必须是有限数字")
    result = float(value)
    if minimum is not None and result < minimum:
        raise CalibrationError(f"{name} 不得小于 {minimum}")
    return result


def _strict_int(name: str, value: object, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise CalibrationError(f"{name} 必须是 {minimum}..{maximum} 的整数")
    return value


@dataclass(frozen=True)
class LiftTrial:
    """一次 300 ms 起升试验的完整观测记录。"""

    pwm: int
    repeat: int
    start_delay_s: float
    displacement_mm: float
    speed_mm_s: float
    coast_mm: float
    peak_current_raw: int
    direction_consistent: bool
    success: bool

    def __post_init__(self) -> None:
        _strict_int("pwm", self.pwm, 0, 100)
        _strict_int("repeat", self.repeat, 1, 3)
        _finite_number("start_delay_s", self.start_delay_s, minimum=0)
        _finite_number("displacement_mm", self.displacement_mm)
        _finite_number("speed_mm_s", self.speed_mm_s)
        _finite_number("coast_mm", self.coast_mm, minimum=0)
        _strict_int("peak_current_raw", self.peak_current_raw, 0, 65535)
        if type(self.direction_consistent) is not bool or type(self.success) is not bool:
            raise CalibrationError("试验方向和成功标志必须是 bool")


@dataclass(frozen=True)
class LowerTrial:
    """一次 150 ms 下降阀脉冲及随后 700 ms 观察记录。"""

    valve: int
    displacement_mm: float
    response_delay_s: float
    direction_consistent: bool
    success: bool

    def __post_init__(self) -> None:
        _strict_int("valve", self.valve, 0, 255)
        _finite_number("displacement_mm", self.displacement_mm, minimum=0)
        _finite_number("response_delay_s", self.response_delay_s, minimum=0)
        if type(self.direction_consistent) is not bool or type(self.success) is not bool:
            raise CalibrationError("下降试验方向和成功标志必须是 bool")


@dataclass(frozen=True)
class LiftCalibrationResult:
    min_stable_pwm: int
    coarse_pwm: int
    response_delay_s: float
    max_coast_mm: float
    peak_current_by_pwm: Mapping[int, int]
    trials: tuple[LiftTrial, ...]


@dataclass(frozen=True)
class LowerCalibrationResult:
    min_start_valve: int
    comfortable_valve: int | None
    trials: tuple[LowerTrial, ...]

    def confirm_comfortable(self, valve: int) -> "LowerCalibrationResult":
        """由操作者确认已实测且能正常下降的舒适阀值。"""
        measured = {
            trial.valve
            for trial in self.trials
            if trial.success and trial.direction_consistent and trial.displacement_mm >= 1.0
        }
        if type(valve) is not int or valve not in measured:
            raise CalibrationError("舒适阀值必须来自成功的实测值")
        return replace(self, comfortable_valve=valve)


def analyze_lift_trials(trials: tuple[LiftTrial, ...]) -> LiftCalibrationResult:
    """分析严格的 40..80、每级三次计划并选择保守控制参数。"""
    expected = [(pwm, repeat) for pwm in LIFT_PWM_LEVELS for repeat in range(1, 4)]
    actual = [(trial.pwm, trial.repeat) for trial in trials]
    if len(trials) != 27 or actual != expected:
        raise CalibrationError("起升标定必须严格包含按顺序执行的 27 次试验")

    min_stable_pwm: int | None = None
    for pwm in LIFT_PWM_LEVELS:
        group = [trial for trial in trials if trial.pwm == pwm]
        if all(
            trial.success and trial.direction_consistent and trial.displacement_mm >= 1.0
            for trial in group
        ):
            min_stable_pwm = pwm
            break
    if min_stable_pwm is None:
        raise CalibrationError("没有找到三次均稳定起升的 PWM")

    stable_group = [trial for trial in trials if trial.pwm == min_stable_pwm]
    peaks = {
        pwm: max(trial.peak_current_raw for trial in trials if trial.pwm == pwm)
        for pwm in LIFT_PWM_LEVELS
    }
    return LiftCalibrationResult(
        min_stable_pwm=min_stable_pwm,
        coarse_pwm=min(min_stable_pwm + 20, 80),
        response_delay_s=max(trial.start_delay_s for trial in stable_group),
        max_coast_mm=max(trial.coast_mm for trial in trials),
        peak_current_by_pwm=peaks,
        trials=trials,
    )


def analyze_lower_trials(trials: tuple[LowerTrial, ...]) -> LowerCalibrationResult:
    """分析严格的 0x10..0xA0 计划；不会自行猜测舒适阀值。"""
    if len(trials) != len(LOWER_VALVE_LEVELS) or tuple(
        trial.valve for trial in trials
    ) != LOWER_VALVE_LEVELS:
        raise CalibrationError("下降标定必须严格按 0x10..0xA0 执行")
    candidates = [
        trial.valve
        for trial in trials
        if trial.success and trial.direction_consistent and trial.displacement_mm >= 1.0
    ]
    if not candidates:
        raise CalibrationError("没有找到能可靠启动下降的阀值")
    return LowerCalibrationResult(min(candidates), None, trials)


def _validated_sample(sample: HeightSample) -> float:
    if not isinstance(sample, HeightSample) or not sample.valid or sample.height_mm is None:
        raise CalibrationError("标定需要有效高度样本")
    return _finite_number("height_mm", sample.height_mm, minimum=0)


def _session_timeout(name: str, value: object, maximum: float) -> float:
    result = _finite_number(name, value, minimum=0)
    if result <= 0 or result > maximum:
        raise CalibrationError(f"{name} 必须大于零且不得超过 {maximum}")
    return result


def _validate_session_inputs(
    *,
    now: object,
    sample: object,
    feedback: object,
    last_now: float | None,
    sensor_timeout_s: float,
    feedback_timeout_s: float,
    absolute_max_height_mm: float,
) -> tuple[float, float, PumpFeedback]:
    """统一校验标定步进输入；调用方捕获异常后锁失败并返回全零。"""
    timestamp = _finite_number("now", now, minimum=0)
    if last_now is not None and timestamp < last_now:
        raise CalibrationError("标定时钟不得回退")
    if (
        not isinstance(sample, HeightSample)
        or type(sample.valid) is not bool
        or not sample.valid
        or sample.height_mm is None
    ):
        raise CalibrationError("标定需要有效高度样本")
    sample_timestamp = _finite_number(
        "sample.timestamp", sample.timestamp, minimum=0
    )
    sample_age = timestamp - sample_timestamp
    if sample_age < 0 or sample_age - sensor_timeout_s > 1e-12:
        raise CalibrationError("标定高度样本已超时或来自未来")
    if type(sample.raw_count) is not int or not 0 <= sample.raw_count <= 0xFFFFFFFF:
        raise CalibrationError("标定高度 raw_count 不合理")
    height = _finite_number("height_mm", sample.height_mm, minimum=0)
    if height > absolute_max_height_mm:
        raise CalibrationError("标定高度超过绝对上限")

    if not isinstance(feedback, PumpFeedback):
        raise CalibrationError("标定需要 CAN 泵反馈")
    feedback_timestamp = _finite_number(
        "feedback.timestamp", feedback.timestamp, minimum=0
    )
    feedback_age = timestamp - feedback_timestamp
    if feedback_age < 0 or feedback_age - feedback_timeout_s > 1e-12:
        raise CalibrationError("标定 CAN 泵反馈已超时或来自未来")
    if type(feedback.fault_code) is not int or feedback.fault_code != 0:
        raise CalibrationError(f"标定 CAN 泵反馈故障码 {feedback.fault_code}")
    if (
        type(feedback.current_raw) is not int
        or not -32768 <= feedback.current_raw <= 32767
    ):
        raise CalibrationError("标定 CAN 泵电流不合理")
    if (
        type(feedback.lower_current_raw) is not int
        or not 0 <= feedback.lower_current_raw <= 255
    ):
        raise CalibrationError("标定下降电流不合理")
    return timestamp, height, feedback


class LiftCalibrationSession:
    """确定性起升标定会话；一次调用最多推进一个相位边界。"""

    def __init__(
        self,
        *,
        direction_tolerance_mm: float = 0.5,
        sensor_timeout_s: float = 0.1,
        feedback_timeout_s: float = 0.15,
        absolute_max_height_mm: float = 2900.0,
    ) -> None:
        self._direction_tolerance_mm = _finite_number(
            "direction_tolerance_mm", direction_tolerance_mm, minimum=0
        )
        self._index = 0
        self._trials: list[LiftTrial] = []
        self._sensor_timeout_s = _session_timeout(
            "sensor_timeout_s", sensor_timeout_s, 0.1
        )
        self._feedback_timeout_s = _session_timeout(
            "feedback_timeout_s", feedback_timeout_s, 0.15
        )
        self._absolute_max_height_mm = _finite_number(
            "absolute_max_height_mm", absolute_max_height_mm, minimum=0.001
        )
        if self._absolute_max_height_mm > 2900.0:
            raise CalibrationError("absolute_max_height_mm 不得超过 2900")
        self.failed = False
        self.fault_reason: str | None = None
        self._last_now: float | None = None
        self._active_started_at: float | None = None
        self._start_height = 0.0
        self._stop_height: float | None = None
        self._lowest_height = 0.0
        self._first_movement_at: float | None = None
        self._peak_current = 0

    @property
    def done(self) -> bool:
        return self._index >= 27

    @property
    def trials(self) -> tuple[LiftTrial, ...]:
        return tuple(self._trials)

    def step(
        self,
        *,
        now: float,
        sample: HeightSample,
        feedback: PumpFeedback | None,
        lift_authorized: bool,
    ) -> PumpCommand:
        """推进 300 ms 通电与 700 ms 稳定阶段；掉授权立即丢弃本次。"""
        if type(lift_authorized) is not bool:
            raise CalibrationError("lift_authorized 必须是 bool")
        if not lift_authorized:
            self._reset_active()
            return PumpCommand.safe_stop()
        if self.failed:
            return PumpCommand.safe_stop()
        try:
            timestamp, height, checked_feedback = _validate_session_inputs(
                now=now,
                sample=sample,
                feedback=feedback,
                last_now=self._last_now,
                sensor_timeout_s=self._sensor_timeout_s,
                feedback_timeout_s=self._feedback_timeout_s,
                absolute_max_height_mm=self._absolute_max_height_mm,
            )
        except CalibrationError as exc:
            return self._fail(str(exc))
        self._last_now = timestamp
        if height >= self._absolute_max_height_mm:
            return self._fail("起升标定高度达到绝对上限")
        self.fault_reason = None
        if self.done:
            return PumpCommand.safe_stop()
        if self._active_started_at is None:
            self._begin(timestamp, height, checked_feedback)
            return PumpCommand(interlock=True, lift_pwm=self._current_pwm())

        elapsed = timestamp - self._active_started_at
        if elapsed < 0:
            raise CalibrationError("标定时钟不得回退")
        if height < self._start_height - self._direction_tolerance_mm:
            return self._fail("起升标定期间高度方向反向")
        self._observe(timestamp, height, checked_feedback)
        # 调用时钟是浮点数，边界比较保留皮秒量级容差，避免 0.3 被表示为
        # 0.299999999999 而意外多通电一个控制周期。
        if elapsed + 1e-12 < 0.3:
            return PumpCommand(interlock=True, lift_pwm=self._current_pwm())
        if self._stop_height is None:
            self._stop_height = height
        if elapsed + 1e-12 < 1.0:
            return PumpCommand.safe_stop()

        self._finish(timestamp, height)
        if self.done:
            return PumpCommand.safe_stop()
        self._begin(timestamp, height, checked_feedback)
        return PumpCommand(interlock=True, lift_pwm=self._current_pwm())

    def _current_pwm(self) -> int:
        return LIFT_PWM_LEVELS[self._index // 3]

    def _begin(self, now: float, height: float, feedback: PumpFeedback | None) -> None:
        self._active_started_at = now
        self._start_height = height
        self._stop_height = None
        self._lowest_height = height
        self._first_movement_at = None
        # 线上的泵电流是有符号原值；标定峰值用于后续过流保护，必须记录幅值。
        self._peak_current = abs(feedback.current_raw) if feedback is not None else 0

    def _observe(
        self, now: float, height: float, feedback: PumpFeedback | None
    ) -> None:
        self._lowest_height = min(self._lowest_height, height)
        if feedback is not None:
            self._peak_current = max(self._peak_current, abs(feedback.current_raw))
        if self._first_movement_at is None and height - self._start_height >= 0.1:
            self._first_movement_at = now

    def _finish(self, now: float, height: float) -> None:
        assert self._active_started_at is not None
        assert self._stop_height is not None
        displacement = height - self._start_height
        direction_ok = self._lowest_height >= self._start_height - self._direction_tolerance_mm
        delay = (
            self._first_movement_at - self._active_started_at
            if self._first_movement_at is not None
            else 0.3
        )
        self._trials.append(
            LiftTrial(
                pwm=self._current_pwm(),
                repeat=self._index % 3 + 1,
                start_delay_s=min(max(delay, 0.0), 0.3),
                displacement_mm=displacement,
                speed_mm_s=max(0.0, self._stop_height - self._start_height) / 0.3,
                coast_mm=max(0.0, height - self._stop_height),
                peak_current_raw=self._peak_current,
                direction_consistent=direction_ok,
                success=direction_ok and displacement >= 1.0,
            )
        )
        self._index += 1
        self._reset_active()

    def _reset_active(self) -> None:
        self._active_started_at = None
        self._stop_height = None
        self._first_movement_at = None

    def _fail(self, reason: str) -> PumpCommand:
        self.failed = True
        self.fault_reason = reason
        self._reset_active()
        return PumpCommand.safe_stop()


class LowerCalibrationSession:
    """确定性下降标定会话；起升字段在所有相位恒为零。"""

    def __init__(
        self,
        *,
        direction_tolerance_mm: float = 0.5,
        sensor_timeout_s: float = 0.1,
        feedback_timeout_s: float = 0.15,
        absolute_max_height_mm: float = 2900.0,
    ) -> None:
        self._direction_tolerance_mm = _finite_number(
            "direction_tolerance_mm", direction_tolerance_mm, minimum=0
        )
        self._index = 0
        self._trials: list[LowerTrial] = []
        self._sensor_timeout_s = _session_timeout(
            "sensor_timeout_s", sensor_timeout_s, 0.1
        )
        self._feedback_timeout_s = _session_timeout(
            "feedback_timeout_s", feedback_timeout_s, 0.15
        )
        self._absolute_max_height_mm = _finite_number(
            "absolute_max_height_mm", absolute_max_height_mm, minimum=0.001
        )
        if self._absolute_max_height_mm > 2900.0:
            raise CalibrationError("absolute_max_height_mm 不得超过 2900")
        self.failed = False
        self.fault_reason: str | None = None
        self._last_now: float | None = None
        self._active_started_at: float | None = None
        self._start_height = 0.0
        self._highest_height = 0.0
        self._first_movement_at: float | None = None

    @property
    def done(self) -> bool:
        return self._index >= len(LOWER_VALVE_LEVELS)

    @property
    def trials(self) -> tuple[LowerTrial, ...]:
        return tuple(self._trials)

    def step(
        self,
        *,
        now: float,
        sample: HeightSample,
        feedback: PumpFeedback | None,
        lower_authorized: bool,
    ) -> PumpCommand:
        """推进 150 ms 阀脉冲与随后 700 ms 观察；掉授权立即全零。"""
        if type(lower_authorized) is not bool:
            raise CalibrationError("lower_authorized 必须是 bool")
        if not lower_authorized:
            self._reset_active()
            return PumpCommand.safe_stop()
        if self.failed:
            return PumpCommand.safe_stop()
        try:
            timestamp, height, _checked_feedback = _validate_session_inputs(
                now=now,
                sample=sample,
                feedback=feedback,
                last_now=self._last_now,
                sensor_timeout_s=self._sensor_timeout_s,
                feedback_timeout_s=self._feedback_timeout_s,
                absolute_max_height_mm=self._absolute_max_height_mm,
            )
        except CalibrationError as exc:
            return self._fail(str(exc))
        self._last_now = timestamp
        self.fault_reason = None
        if self.done:
            return PumpCommand.safe_stop()
        if self._active_started_at is None:
            self._begin(timestamp, height)
            return PumpCommand(interlock=True, lower_valve=LOWER_VALVE_LEVELS[self._index])

        elapsed = timestamp - self._active_started_at
        if elapsed < 0:
            raise CalibrationError("标定时钟不得回退")
        if height > self._start_height + self._direction_tolerance_mm:
            return self._fail("下降标定期间高度方向反向")
        self._highest_height = max(self._highest_height, height)
        if self._first_movement_at is None and self._start_height - height >= 0.1:
            self._first_movement_at = timestamp
        if elapsed + 1e-12 < 0.15:
            return PumpCommand(interlock=True, lower_valve=LOWER_VALVE_LEVELS[self._index])
        if elapsed + 1e-12 < 0.85:
            return PumpCommand.safe_stop()

        displacement = self._start_height - height
        direction_ok = self._highest_height <= self._start_height + self._direction_tolerance_mm
        delay = (
            self._first_movement_at - self._active_started_at
            if self._first_movement_at is not None
            else 0.15
        )
        self._trials.append(
            LowerTrial(
                valve=LOWER_VALVE_LEVELS[self._index],
                displacement_mm=max(0.0, displacement),
                response_delay_s=min(max(delay, 0.0), 0.15),
                direction_consistent=direction_ok,
                success=direction_ok and displacement >= 1.0,
            )
        )
        self._index += 1
        self._reset_active()
        if self.done:
            return PumpCommand.safe_stop()
        self._begin(timestamp, height)
        return PumpCommand(interlock=True, lower_valve=LOWER_VALVE_LEVELS[self._index])

    def _begin(self, now: float, height: float) -> None:
        self._active_started_at = now
        self._start_height = height
        self._highest_height = height
        self._first_movement_at = None

    def _reset_active(self) -> None:
        self._active_started_at = None
        self._first_movement_at = None

    def _fail(self, reason: str) -> PumpCommand:
        self.failed = True
        self.fault_reason = reason
        self._reset_active()
        return PumpCommand.safe_stop()


@dataclass(frozen=True)
class CalibrationBundle:
    """运行控制所需的最小标定摘要；schema v1 不接受未知字段。"""

    min_stable_pwm: int
    coarse_pwm: int
    response_delay_s: float
    max_coast_mm: float
    peak_current_by_pwm: Mapping[int, int]
    lower_min_start_valve: int
    lower_comfortable_valve: int
    soft_upper_limit_mm: float | None = None

    def __post_init__(self) -> None:
        _strict_int("min_stable_pwm", self.min_stable_pwm, 40, 80)
        if self.min_stable_pwm not in LIFT_PWM_LEVELS:
            raise CalibrationError("min_stable_pwm 必须来自 40..80 的实测 PWM 级")
        _strict_int("coarse_pwm", self.coarse_pwm, self.min_stable_pwm, 80)
        if self.coarse_pwm != min(self.min_stable_pwm + 20, 80):
            raise CalibrationError("coarse_pwm 必须等于 min_stable_pwm+20 并封顶 80")
        response_delay = _finite_number(
            "response_delay_s", self.response_delay_s, minimum=0
        )
        if response_delay > 0.3:
            raise CalibrationError("response_delay_s 不得超过起升试验的 0.3 秒")
        _finite_number("max_coast_mm", self.max_coast_mm, minimum=0)
        _strict_int("lower_min_start_valve", self.lower_min_start_valve, 0x10, 0xA0)
        _strict_int("lower_comfortable_valve", self.lower_comfortable_valve, 0x10, 0xA0)
        if self.lower_min_start_valve not in LOWER_VALVE_LEVELS:
            raise CalibrationError("lower_min_start_valve 必须来自离散实测阀值")
        if (
            self.lower_comfortable_valve not in LOWER_VALVE_LEVELS
            or self.lower_comfortable_valve < self.lower_min_start_valve
        ):
            raise CalibrationError(
                "lower_comfortable_valve 必须来自不低于启动值的离散实测阀值"
            )
        if not isinstance(self.peak_current_by_pwm, Mapping) or set(
            self.peak_current_by_pwm
        ) != set(LIFT_PWM_LEVELS):
            raise CalibrationError("peak_current_by_pwm 必须包含每个起升 PWM")
        peak_copy = dict(self.peak_current_by_pwm)
        for pwm, current in peak_copy.items():
            _strict_int("peak_current PWM", pwm, 40, 80)
            _strict_int("peak_current_raw", current, 0, 65535)
        # frozen dataclass 仍无法冻结调用方传入的 dict；复制后用只读代理阻止
        # 运行期修改使控制器和持久化 schema 漂移。
        object.__setattr__(
            self, "peak_current_by_pwm", MappingProxyType(peak_copy)
        )
        if self.soft_upper_limit_mm is not None:
            limit = _finite_number("soft_upper_limit_mm", self.soft_upper_limit_mm, minimum=0.001)
            if limit > 2900.0:
                raise CalibrationError("soft_upper_limit_mm 不得超过 2900")

    @classmethod
    def from_results(
        cls,
        lift: LiftCalibrationResult,
        lower: LowerCalibrationResult,
        *,
        soft_upper_limit_mm: float | None = None,
    ) -> "CalibrationBundle":
        if lower.comfortable_valve is None:
            raise CalibrationError("保存前必须由操作者确认舒适下降阀值")
        return cls(
            min_stable_pwm=lift.min_stable_pwm,
            coarse_pwm=lift.coarse_pwm,
            response_delay_s=lift.response_delay_s,
            max_coast_mm=lift.max_coast_mm,
            peak_current_by_pwm=dict(lift.peak_current_by_pwm),
            lower_min_start_valve=lower.min_start_valve,
            lower_comfortable_valve=lower.comfortable_valve,
            soft_upper_limit_mm=soft_upper_limit_mm,
        )

    def to_json_object(self) -> dict[str, Any]:
        return {
            "schema_version": CALIBRATION_SCHEMA_VERSION,
            "min_stable_pwm": self.min_stable_pwm,
            "coarse_pwm": self.coarse_pwm,
            "response_delay_s": self.response_delay_s,
            "max_coast_mm": self.max_coast_mm,
            "peak_current_by_pwm": {
                str(pwm): current for pwm, current in sorted(self.peak_current_by_pwm.items())
            },
            "lower_min_start_valve": self.lower_min_start_valve,
            "lower_comfortable_valve": self.lower_comfortable_valve,
            "soft_upper_limit_mm": self.soft_upper_limit_mm,
        }

    @classmethod
    def from_json_object(cls, raw: object) -> "CalibrationBundle":
        fields = {
            "schema_version",
            "min_stable_pwm",
            "coarse_pwm",
            "response_delay_s",
            "max_coast_mm",
            "peak_current_by_pwm",
            "lower_min_start_valve",
            "lower_comfortable_valve",
            "soft_upper_limit_mm",
        }
        if type(raw) is not dict or set(raw) != fields:
            raise CalibrationError("标定文件字段不完整或包含未知字段")
        if (
            type(raw["schema_version"]) is not int
            or raw["schema_version"] != CALIBRATION_SCHEMA_VERSION
        ):
            raise CalibrationError("不支持的标定 schema_version")
        peaks_raw = raw["peak_current_by_pwm"]
        if type(peaks_raw) is not dict:
            raise CalibrationError("peak_current_by_pwm 必须是对象")
        if set(peaks_raw) != {str(pwm) for pwm in LIFT_PWM_LEVELS}:
            raise CalibrationError("peak_current_by_pwm 键必须是规范的实测 PWM 字符串")
        try:
            peaks = {int(pwm): current for pwm, current in peaks_raw.items()}
        except (TypeError, ValueError) as exc:
            raise CalibrationError("peak_current_by_pwm 键必须是十进制 PWM") from exc
        return cls(
            min_stable_pwm=raw["min_stable_pwm"],
            coarse_pwm=raw["coarse_pwm"],
            response_delay_s=raw["response_delay_s"],
            max_coast_mm=raw["max_coast_mm"],
            peak_current_by_pwm=peaks,
            lower_min_start_valve=raw["lower_min_start_valve"],
            lower_comfortable_valve=raw["lower_comfortable_valve"],
            soft_upper_limit_mm=raw["soft_upper_limit_mm"],
        )


class CalibrationStore:
    """采用同目录临时文件和 ``os.replace`` 原子保存标定摘要。"""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = (
            Path(path)
            if path is not None
            else Path.home()
            / ".local"
            / "state"
            / "agv-lift-height-control"
            / "calibration.json"
        )

    def save(self, bundle: CalibrationBundle) -> None:
        if not isinstance(bundle, CalibrationBundle):
            raise TypeError("bundle 必须是 CalibrationBundle")
        # 写前用与 load 完全相同的解析器重新验证，确保绝不原子写入一个随后
        # 无法加载的对象；同时防御通过 object.__setattr__ 等方式破坏 frozen 实例。
        validated = CalibrationBundle.from_json_object(bundle.to_json_object())
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                json.dump(
                    validated.to_json_object(),
                    stream,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                    allow_nan=False,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self.path)
            temporary_path = None
        except (OSError, TypeError, ValueError) as exc:
            raise CalibrationError(f"无法原子保存标定文件: {exc}") from exc
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def load(self) -> CalibrationBundle:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            return CalibrationBundle.from_json_object(raw)
        except CalibrationError:
            raise
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
            raise CalibrationError(f"无法读取标定文件: {exc}") from exc


class UpperLimitSurvey:
    """分段测量上限，连续起升不超过配置时间且不自行持久化结果。"""

    def __init__(
        self,
        config: ControlConfig,
        calibration: CalibrationBundle,
        *,
        temporary_max_height_mm: float | None,
    ) -> None:
        if not isinstance(config, ControlConfig):
            raise TypeError("config 必须是 ControlConfig")
        if not isinstance(calibration, CalibrationBundle):
            raise TypeError("calibration 必须是 CalibrationBundle")
        if temporary_max_height_mm is None:
            raise CalibrationError("上限测量必须提供人工临时最大高度")
        temporary = _finite_number(
            "临时最大高度", temporary_max_height_mm, minimum=0.001
        )
        if temporary > config.absolute_max_height_mm or temporary > 2900.0:
            raise CalibrationError("临时最大高度不得超过 2900 mm 绝对上限")
        self.config = config
        self.calibration = calibration
        self.temporary_max_height_mm = temporary
        self.highest_observed_mm: float | None = None
        self.limit_reached = False
        self.failed = False
        self.fault_reason: str | None = None
        self._on_started_at: float | None = None
        self._pause_until: float | None = None
        self._last_now: float | None = None

    @property
    def suggested_soft_limit_mm(self) -> float:
        """以最高观测值减去 50 mm 或两倍滑行距离中的较大者。"""
        if self.highest_observed_mm is None:
            raise CalibrationError("尚无上限测量值，不能生成软限位建议")
        margin = max(50.0, 2.0 * self.calibration.max_coast_mm)
        suggestion = min(
            self.highest_observed_mm - margin,
            self.temporary_max_height_mm,
            self.config.absolute_max_height_mm,
            2900.0,
        )
        if suggestion <= 0:
            raise CalibrationError("观测高度不足以生成大于零的软限位")
        return suggestion

    def step(
        self,
        *,
        now: float,
        sample: HeightSample,
        lift_authorized: bool,
    ) -> PumpCommand:
        """推进测量周期；撤权立即归零，并把活动段转换为完整强制暂停。"""
        if type(lift_authorized) is not bool:
            raise CalibrationError("lift_authorized 必须是 bool")
        if self.limit_reached or self.failed:
            # 临时/绝对上限或输入故障是本次测量的终止门禁。
            return PumpCommand.safe_stop()
        if not lift_authorized:
            try:
                timestamp = _finite_number("now", now, minimum=0)
                if self._last_now is not None and timestamp < self._last_now:
                    raise CalibrationError("上限测量时钟不得回退")
            except CalibrationError as exc:
                self._on_started_at = None
                self.failed = True
                self.fault_reason = str(exc)
                return PumpCommand.safe_stop()
            self._last_now = timestamp
            if self._on_started_at is not None:
                mandatory_pause_until = timestamp + self.config.survey_pause_s
                self._pause_until = max(
                    self._pause_until or mandatory_pause_until,
                    mandatory_pause_until,
                )
            self._on_started_at = None
            return PumpCommand.safe_stop()
        try:
            timestamp = _finite_number("now", now, minimum=0)
            if self._last_now is not None and timestamp < self._last_now:
                raise CalibrationError("上限测量时钟不得回退")
            height = self._validate_survey_sample(timestamp, sample)
        except CalibrationError as exc:
            self._on_started_at = None
            self.failed = True
            self.fault_reason = str(exc)
            return PumpCommand.safe_stop()
        self._last_now = timestamp
        self.fault_reason = None
        self.highest_observed_mm = (
            height
            if self.highest_observed_mm is None
            else max(self.highest_observed_mm, height)
        )
        if height >= min(self.temporary_max_height_mm, self.config.absolute_max_height_mm):
            self.limit_reached = True
            self._on_started_at = None
            return PumpCommand.safe_stop()
        if self._on_started_at is None:
            if self._pause_until is not None and timestamp + 1e-12 < self._pause_until:
                return PumpCommand.safe_stop()
            self._on_started_at = timestamp
            self._pause_until = None
            return PumpCommand(
                interlock=True, lift_pwm=self.calibration.min_stable_pwm
            )
        if timestamp - self._on_started_at + 1e-12 < self.config.survey_max_on_s:
            return PumpCommand(
                interlock=True, lift_pwm=self.calibration.min_stable_pwm
            )
        self._on_started_at = None
        self._pause_until = timestamp + self.config.survey_pause_s
        return PumpCommand.safe_stop()

    def _validate_survey_sample(self, now: float, sample: object) -> float:
        """校验测量样本与新鲜度；失败由 step 锁存为结束态而不是向上抛出。"""
        if (
            not isinstance(sample, HeightSample)
            or type(sample.valid) is not bool
            or not sample.valid
            or sample.height_mm is None
        ):
            raise CalibrationError("上限测量需要有效高度样本")
        _finite_number("sample.timestamp", sample.timestamp, minimum=0)
        age = now - float(sample.timestamp)
        if age < 0 or age - self.config.sensor_timeout_s > 1e-12:
            raise CalibrationError("上限测量高度样本已超时或来自未来")
        if type(sample.raw_count) is not int or not 0 <= sample.raw_count <= 0xFFFFFFFF:
            raise CalibrationError("上限测量 raw_count 不合理")
        height = _finite_number("height_mm", sample.height_mm, minimum=0)
        if height > self.config.absolute_max_height_mm:
            raise CalibrationError("上限测量高度超过绝对上限")
        return height

    def confirm(
        self,
        bundle: CalibrationBundle,
        *,
        store: CalibrationStore | None = None,
    ) -> CalibrationBundle:
        """操作者显式确认建议值；仅此方法可选择写入标定存储。"""
        if not isinstance(bundle, CalibrationBundle):
            raise TypeError("bundle 必须是 CalibrationBundle")
        if self.failed:
            raise CalibrationError("上限测量已失败，禁止确认或持久化半成品结果")
        updated = replace(bundle, soft_upper_limit_mm=self.suggested_soft_limit_mm)
        if store is not None:
            if not isinstance(store, CalibrationStore):
                raise TypeError("store 必须是 CalibrationStore")
            store.save(updated)
        return updated
