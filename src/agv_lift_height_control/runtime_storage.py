"""首次起升标定跨命令恢复所需的严格版本化草稿。"""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

from .calibration import (
    CalibrationError,
    LiftCalibrationResult,
    LiftTrial,
    analyze_lift_trials,
)

DRAFT_SCHEMA_VERSION = 1


class CalibrationDraftStore:
    """原子保存完整起升结果和 27 次原始试验，不接受未知字段。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save_lift(self, result: LiftCalibrationResult) -> None:
        validated = _validate_result(result)
        raw = _result_to_json(validated)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                "w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                temporary = Path(stream.name)
                json.dump(raw, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            temporary = None
        except (OSError, TypeError, ValueError) as exc:
            raise CalibrationError(f"无法原子保存起升标定草稿: {exc}") from exc
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

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
