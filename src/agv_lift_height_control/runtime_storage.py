"""首次起升标定跨命令恢复所需的严格版本化草稿。"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from hashlib import sha256
from math import isfinite
from pathlib import Path
from typing import Any

from .calibration import (
    CalibrationBundle,
    CalibrationError,
    LiftCalibrationResult,
    LiftTrial,
    LowerCalibrationResult,
    LowerTrial,
    analyze_lift_trials,
    analyze_lower_trials,
)

DRAFT_SCHEMA_VERSION = 1


class CalibrationDraftStore:
    """原子保存完整起升结果和 27 次原始试验，不接受未知字段。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save_lift(self, result: LiftCalibrationResult) -> None:
        validated = _validate_result(result)
        _atomic_json_save(self.path, _result_to_json(validated), "起升标定草稿")

    def load_lift(self) -> LiftCalibrationResult:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            return _result_from_json(raw)
        except CalibrationError:
            raise
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
            raise CalibrationError(f"无法读取起升标定草稿: {exc}") from exc


def _validate_result(result: object) -> LiftCalibrationResult:
    if not isinstance(result, LiftCalibrationResult):
        raise CalibrationError("草稿必须是 LiftCalibrationResult")
    analyzed = analyze_lift_trials(result.trials)
    if (
        result.min_stable_pwm != analyzed.min_stable_pwm
        or result.coarse_pwm != analyzed.coarse_pwm
        or result.response_delay_s != analyzed.response_delay_s
        or result.max_coast_mm != analyzed.max_coast_mm
        or dict(result.peak_current_by_pwm) != dict(analyzed.peak_current_by_pwm)
    ):
        raise CalibrationError("起升标定摘要与完整 trials 不一致")
    return analyzed


def _result_to_json(result: LiftCalibrationResult) -> dict[str, object]:
    return {
        "schema_version": DRAFT_SCHEMA_VERSION,
        "lift": {
            "min_stable_pwm": result.min_stable_pwm,
            "coarse_pwm": result.coarse_pwm,
            "response_delay_s": result.response_delay_s,
            "max_coast_mm": result.max_coast_mm,
            "peak_current_by_pwm": {
                str(key): value for key, value in sorted(result.peak_current_by_pwm.items())
            },
            "trials": [
                {
                    "pwm": trial.pwm,
                    "repeat": trial.repeat,
                    "start_delay_s": trial.start_delay_s,
                    "displacement_mm": trial.displacement_mm,
                    "speed_mm_s": trial.speed_mm_s,
                    "coast_mm": trial.coast_mm,
                    "peak_current_raw": trial.peak_current_raw,
                    "direction_consistent": trial.direction_consistent,
                    "success": trial.success,
                }
                for trial in result.trials
            ],
        },
    }


def _result_from_json(raw: object) -> LiftCalibrationResult:
    if type(raw) is not dict or set(raw) != {"schema_version", "lift"}:
        raise CalibrationError("起升草稿字段不完整或包含未知字段")
    if type(raw["schema_version"]) is not int or raw["schema_version"] != DRAFT_SCHEMA_VERSION:
        raise CalibrationError("不支持的起升草稿 schema_version")
    lift = raw["lift"]
    lift_fields = {
        "min_stable_pwm",
        "coarse_pwm",
        "response_delay_s",
        "max_coast_mm",
        "peak_current_by_pwm",
        "trials",
    }
    if type(lift) is not dict or set(lift) != lift_fields:
        raise CalibrationError("起升草稿 lift 字段不完整或包含未知字段")
    trials_raw = lift["trials"]
    if type(trials_raw) is not list:
        raise CalibrationError("起升草稿 trials 必须是数组")
    trial_fields = {
        "pwm",
        "repeat",
        "start_delay_s",
        "displacement_mm",
        "speed_mm_s",
        "coast_mm",
        "peak_current_raw",
        "direction_consistent",
        "success",
    }
    trials: list[LiftTrial] = []
    for trial in trials_raw:
        if type(trial) is not dict or set(trial) != trial_fields:
            raise CalibrationError("起升草稿 trial 字段不完整或包含未知字段")
        trials.append(LiftTrial(**trial))
    peaks = lift["peak_current_by_pwm"]
    if type(peaks) is not dict:
        raise CalibrationError("起升草稿 peak_current_by_pwm 必须是对象")
    try:
        result = LiftCalibrationResult(
            min_stable_pwm=lift["min_stable_pwm"],
            coarse_pwm=lift["coarse_pwm"],
            response_delay_s=lift["response_delay_s"],
            max_coast_mm=lift["max_coast_mm"],
            peak_current_by_pwm={int(key): value for key, value in peaks.items()},
            trials=tuple(trials),
        )
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"起升草稿字段类型错误: {exc}") from exc
    return _validate_result(result)


LOWER_DRAFT_SCHEMA_VERSION = 2
SURVEY_DRAFT_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class LowerCalibrationDraft:
    """下降试验结果及其所依据的完整起升标定指纹。"""

    result: LowerCalibrationResult
    lift_fingerprint: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "result", _validate_lower_result(self.result))
        _validate_fingerprint("lift_fingerprint", self.lift_fingerprint)


class LowerCalibrationDraftStore:
    """保存完整下降试验；动作阶段绝不包含提前猜测的舒适阀值。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save(self, result: LowerCalibrationResult, *, lift_fingerprint: str) -> None:
        draft = LowerCalibrationDraft(result=result, lift_fingerprint=lift_fingerprint)
        raw = {
            "schema_version": LOWER_DRAFT_SCHEMA_VERSION,
            "lower": {
                "min_start_valve": draft.result.min_start_valve,
                "comfortable_valve": None,
                "lift_fingerprint": draft.lift_fingerprint,
                "trials": [
                    {
                        "valve": trial.valve,
                        "displacement_mm": trial.displacement_mm,
                        "response_delay_s": trial.response_delay_s,
                        "direction_consistent": trial.direction_consistent,
                        "success": trial.success,
                    }
                    for trial in draft.result.trials
                ],
            },
        }
        _atomic_json_save(self.path, raw, "下降标定草稿")

    def load(self) -> LowerCalibrationDraft:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"无法读取下降标定草稿: {exc}") from exc
        if type(raw) is not dict or set(raw) != {"schema_version", "lower"}:
            raise CalibrationError("下降草稿字段不完整或包含未知字段")
        schema_version = raw["schema_version"]
        if type(schema_version) is not int:
            raise CalibrationError("不支持的下降草稿 schema_version")
        if schema_version == 1:
            raise CalibrationError("旧版下降草稿缺少起升指纹；请重新执行下降标定")
        if schema_version != LOWER_DRAFT_SCHEMA_VERSION:
            raise CalibrationError("不支持的下降草稿 schema_version")
        lower = raw["lower"]
        expected = {
            "min_start_valve",
            "comfortable_valve",
            "lift_fingerprint",
            "trials",
        }
        if type(lower) is not dict or set(lower) != expected:
            raise CalibrationError("下降草稿 lower 字段不完整或包含未知字段")
        if lower["comfortable_valve"] is not None:
            raise CalibrationError("下降动作草稿不得包含提前确认的舒适阀值")
        trials_raw = lower["trials"]
        trial_fields = {
            "valve",
            "displacement_mm",
            "response_delay_s",
            "direction_consistent",
            "success",
        }
        if type(trials_raw) is not list:
            raise CalibrationError("下降草稿 trials 必须是数组")
        trials: list[LowerTrial] = []
        try:
            for trial in trials_raw:
                if type(trial) is not dict or set(trial) != trial_fields:
                    raise CalibrationError("下降草稿 trial 字段不完整或包含未知字段")
                trials.append(LowerTrial(**trial))
            result = LowerCalibrationResult(
                min_start_valve=lower["min_start_valve"],
                comfortable_valve=None,
                trials=tuple(trials),
            )
        except (TypeError, ValueError) as exc:
            raise CalibrationError(f"下降草稿字段类型错误: {exc}") from exc
        return LowerCalibrationDraft(
            result=result,
            lift_fingerprint=lower["lift_fingerprint"],
        )


def _validate_lower_result(result: object) -> LowerCalibrationResult:
    if not isinstance(result, LowerCalibrationResult):
        raise CalibrationError("下降草稿必须是 LowerCalibrationResult")
    if result.comfortable_valve is not None:
        raise CalibrationError("下降动作草稿不得提前确认舒适阀值")
    analyzed = analyze_lower_trials(result.trials)
    if result.min_start_valve != analyzed.min_start_valve:
        raise CalibrationError("下降标定摘要与完整 trials 不一致")
    return analyzed


@dataclass(frozen=True)
class SurveyDraft:
    """一次上限测量的只读结果和所依据标定包指纹。"""

    highest_observed_mm: float
    suggested_soft_limit_mm: float
    temporary_max_height_mm: float
    calibration_fingerprint: str

    def __post_init__(self) -> None:
        for name in (
            "highest_observed_mm",
            "suggested_soft_limit_mm",
            "temporary_max_height_mm",
        ):
            value = getattr(self, name)
            if type(value) not in {int, float} or not isfinite(float(value)) or value <= 0:
                raise CalibrationError(f"{name} 必须是有限正数")
            object.__setattr__(self, name, float(value))
        if self.suggested_soft_limit_mm > min(
            self.highest_observed_mm, self.temporary_max_height_mm, 2900.0
        ):
            raise CalibrationError("上限建议不得超过观测值、临时上限或绝对上限")
        fingerprint = self.calibration_fingerprint
        if (
            type(fingerprint) is not str
            or len(fingerprint) != 64
            or any(character not in "0123456789abcdef" for character in fingerprint)
        ):
            raise CalibrationError("calibration_fingerprint 必须是 SHA-256 小写十六进制")


class SurveyDraftStore:
    """严格、原子保存上限观测，供独立无硬件确认命令使用。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save(self, draft: SurveyDraft) -> None:
        if not isinstance(draft, SurveyDraft):
            raise CalibrationError("上限草稿必须是 SurveyDraft")
        raw = {
            "schema_version": SURVEY_DRAFT_SCHEMA_VERSION,
            "survey": {
                "highest_observed_mm": draft.highest_observed_mm,
                "suggested_soft_limit_mm": draft.suggested_soft_limit_mm,
                "temporary_max_height_mm": draft.temporary_max_height_mm,
                "calibration_fingerprint": draft.calibration_fingerprint,
            },
        }
        _atomic_json_save(self.path, raw, "上限测量草稿")

    def load(self) -> SurveyDraft:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"无法读取上限测量草稿: {exc}") from exc
        if type(raw) is not dict or set(raw) != {"schema_version", "survey"}:
            raise CalibrationError("上限草稿字段不完整或包含未知字段")
        if (
            type(raw["schema_version"]) is not int
            or raw["schema_version"] != SURVEY_DRAFT_SCHEMA_VERSION
        ):
            raise CalibrationError("不支持的上限草稿 schema_version")
        survey = raw["survey"]
        fields = {
            "highest_observed_mm",
            "suggested_soft_limit_mm",
            "temporary_max_height_mm",
            "calibration_fingerprint",
        }
        if type(survey) is not dict or set(survey) != fields:
            raise CalibrationError("上限草稿 survey 字段不完整或包含未知字段")
        try:
            return SurveyDraft(**survey)
        except (TypeError, ValueError) as exc:
            raise CalibrationError(f"上限草稿字段类型错误: {exc}") from exc


def calibration_fingerprint(bundle: CalibrationBundle) -> str:
    if not isinstance(bundle, CalibrationBundle):
        raise TypeError("bundle 必须是 CalibrationBundle")
    canonical = json.dumps(
        bundle.to_json_object(),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return sha256(canonical).hexdigest()


def lift_calibration_fingerprint(result: LiftCalibrationResult) -> str:
    """对规范化后的完整起升标定结果计算稳定 SHA-256 指纹。"""

    validated = _validate_result(result)
    canonical = json.dumps(
        _result_to_json(validated)["lift"],
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return sha256(canonical).hexdigest()


def _validate_fingerprint(name: str, fingerprint: object) -> None:
    if (
        type(fingerprint) is not str
        or len(fingerprint) != 64
        or any(character not in "0123456789abcdef" for character in fingerprint)
    ):
        raise CalibrationError(f"{name} 必须是 SHA-256 小写十六进制")


def _atomic_json_save(path: Path, raw: dict[str, Any], label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(raw, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    except (OSError, TypeError, ValueError) as exc:
        raise CalibrationError(f"无法原子保存{label}: {exc}") from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
