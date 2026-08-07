import json
from collections.abc import Mapping
from typing import get_type_hints

import pytest

from agv_lift_height_control import HeightSample, PumpFeedback
from agv_lift_height_control.calibration import (
    LIFT_PWM_LEVELS,
    LOWER_VALVE_LEVELS,
    CalibrationBundle,
    CalibrationError,
    CalibrationStore,
    LiftCalibrationSession,
    LiftTrial,
    LowerCalibrationSession,
    LowerTrial,
    analyze_lift_trials,
    analyze_lower_trials,
)


def sample(timestamp: float, height_mm: float, *, valid: bool = True) -> HeightSample:
    return HeightSample(timestamp, 100, height_mm, valid, None if valid else "bad")


def feedback(timestamp: float, current: int = 100, *, fault: int = 0) -> PumpFeedback:
    return PumpFeedback(timestamp, current, fault, 0)


def complete_lift_trials() -> tuple[LiftTrial, ...]:
    trials = []
    for pwm in LIFT_PWM_LEVELS:
        for repeat in range(1, 4):
            moved = 2.0 if pwm >= 50 else 0.5
            trials.append(
                LiftTrial(
                    pwm=pwm,
                    repeat=repeat,
                    start_delay_s=0.12 + repeat / 100,
                    displacement_mm=moved,
                    speed_mm_s=moved / 0.3,
                    coast_mm=float(pwm - 40) / 5,
                    peak_current_raw=pwm * 10 + repeat,
                    direction_consistent=True,
                    success=moved >= 1.0,
                )
            )
    return tuple(trials)


def complete_lower_trials() -> tuple[LowerTrial, ...]:
    return tuple(
        LowerTrial(
            valve=valve,
            displacement_mm=2.0 if valve >= 0x30 else 0.2,
            response_delay_s=0.08,
            direction_consistent=True,
            success=valve >= 0x30,
        )
        for valve in LOWER_VALVE_LEVELS
    )


def test_lift_analysis_requires_exact_plan_and_derives_safe_summary() -> None:
    result = analyze_lift_trials(complete_lift_trials())

    assert LIFT_PWM_LEVELS == tuple(range(40, 81, 5))
    assert result.min_stable_pwm == 50
    assert result.coarse_pwm == 70
    assert result.response_delay_s == pytest.approx(0.15)
    assert result.max_coast_mm == 8.0
    assert result.peak_current_by_pwm[50] == 503

    with pytest.raises(CalibrationError, match="27"):
        analyze_lift_trials(complete_lift_trials()[:-1])


def test_lower_analysis_requires_exact_plan_and_explicit_measured_comfort_value() -> None:
    result = analyze_lower_trials(complete_lower_trials())

    assert LOWER_VALVE_LEVELS == tuple(range(0x10, 0xA1, 0x10))
    assert result.min_start_valve == 0x30
    assert result.comfortable_valve is None
    with pytest.raises(CalibrationError, match="实测"):
        result.confirm_comfortable(0x35)

    confirmed = result.confirm_comfortable(0x50)

    assert confirmed.comfortable_valve == 0x50


def test_lift_session_runs_three_300ms_pulses_per_level_with_700ms_settle() -> None:
    session = LiftCalibrationSession()
    now = 0.0
    height = 100.0

    command = session.step(
        now=now,
        sample=sample(now, height),
        feedback=feedback(now),
        lift_authorized=True,
    )
    assert command.lift_pwm == 40

    for index in range(27):
        command = session.step(
            now=now + 0.3,
            sample=sample(now + 0.3, height + 2.0),
            feedback=feedback(now + 0.3, 200 + index),
            lift_authorized=True,
        )
        assert command.lift_pwm == 0
        command = session.step(
            now=now + 1.0,
            sample=sample(now + 1.0, height + 2.5),
            feedback=feedback(now + 1.0, 250 + index),
            lift_authorized=True,
        )
        if index < 26:
            assert command.lift_pwm == LIFT_PWM_LEVELS[(index + 1) // 3]
        else:
            assert command.lift_pwm == 0
        height += 2.5
        now += 1.0

    assert session.done
    assert len(session.trials) == 27
    assert [(trial.pwm, trial.repeat) for trial in session.trials] == [
        (pwm, repeat) for pwm in LIFT_PWM_LEVELS for repeat in range(1, 4)
    ]


def test_lift_session_authorization_loss_immediately_stops_and_restarts_trial() -> None:
    session = LiftCalibrationSession()

    assert session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=0.1,
        sample=sample(0.1, 10.2),
        feedback=feedback(0.1),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=0.2,
        sample=sample(0.2, 10.2),
        feedback=feedback(0.2),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.trials == ()


def test_lower_session_uses_150ms_pulse_and_700ms_observation() -> None:
    session = LowerCalibrationSession()
    now = 0.0
    height = 200.0

    assert session.step(
        now=now,
        sample=sample(now, height),
        feedback=feedback(now),
        lower_authorized=True,
    ).lower_valve == 0x10

    for index, valve in enumerate(LOWER_VALVE_LEVELS):
        assert session.step(
            now=now + 0.15,
            sample=sample(now + 0.15, height - 1.0),
            feedback=feedback(now + 0.15),
            lower_authorized=True,
        ).lower_valve == 0
        command = session.step(
            now=now + 0.85,
            sample=sample(now + 0.85, height - 2.0),
            feedback=feedback(now + 0.85),
            lower_authorized=True,
        )
        assert command.lift_pwm == 0
        assert command.lower_valve == (LOWER_VALVE_LEVELS[index + 1] if index < 9 else 0)
        height -= 2.0
        now += 0.85

    assert session.done
    assert tuple(trial.valve for trial in session.trials) == LOWER_VALVE_LEVELS


def test_lower_session_authorization_loss_immediately_stops() -> None:
    session = LowerCalibrationSession()
    session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lower_authorized=True,
    )

    command = session.step(
        now=0.05,
        sample=sample(0.05, 99.9),
        feedback=feedback(0.05),
        lower_authorized=False,
    )

    assert command.lift_pwm == 0
    assert command.lower_valve == 0
    assert session.trials == ()


def test_lift_session_expired_sample_latches_failure_during_output() -> None:
    session = LiftCalibrationSession()
    assert session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    ).lift_pwm == 40

    command = session.step(
        now=0.2,
        sample=sample(0.0, 100.2),
        feedback=feedback(0.2),
        lift_authorized=True,
    )

    assert command.lift_pwm == command.lower_valve == 0
    assert session.failed
    assert "超时" in (session.fault_reason or "")
    assert session.step(
        now=0.21,
        sample=sample(0.21, 100.2),
        feedback=feedback(0.21),
        lift_authorized=True,
    ).lift_pwm == 0


def test_lift_session_missing_feedback_fails_closed() -> None:
    session = LiftCalibrationSession()

    command = session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=None,
        lift_authorized=True,
    )

    assert command.lift_pwm == 0
    assert session.failed
    assert "反馈" in (session.fault_reason or "")


def test_lift_session_rejects_absolute_limit_on_first_sample_and_latches_stop() -> None:
    session = LiftCalibrationSession()

    command = session.step(
        now=0.0,
        sample=sample(0.0, 2900.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )

    assert command == command.safe_stop()
    assert session.failed
    assert "绝对上限" in (session.fault_reason or "")
    assert session.step(
        now=0.01,
        sample=sample(0.01, 100.0),
        feedback=feedback(0.01),
        lift_authorized=True,
    ) == command.safe_stop()


def test_lift_session_immediately_latches_reverse_motion_during_active_pulse() -> None:
    session = LiftCalibrationSession(direction_tolerance_mm=0.5)
    assert session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    ).lift_pwm == 40

    command = session.step(
        now=0.1,
        sample=sample(0.1, 98.0),
        feedback=feedback(0.1),
        lift_authorized=True,
    )

    assert command == command.safe_stop()
    assert session.failed
    assert "方向反向" in (session.fault_reason or "")
    assert session.trials == ()
    assert session.step(
        now=1.0,
        sample=sample(1.0, 101.0),
        feedback=feedback(1.0),
        lift_authorized=True,
    ) == command.safe_stop()


def test_lower_session_feedback_fault_during_output_latches_failure() -> None:
    session = LowerCalibrationSession()
    assert session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lower_authorized=True,
    ).lower_valve == 0x10

    command = session.step(
        now=0.05,
        sample=sample(0.05, 99.9),
        feedback=feedback(0.05, fault=7),
        lower_authorized=True,
    )

    assert command.lower_valve == command.lift_pwm == 0
    assert session.failed
    assert "故障码 7" in (session.fault_reason or "")
    assert session.step(
        now=float("nan"),
        sample=HeightSample(float("nan"), None, None, False, "bad"),
        feedback=None,
        lower_authorized=False,
    ).lower_valve == 0
    assert session.failed


def test_lower_session_immediately_latches_reverse_motion_and_stays_stopped() -> None:
    session = LowerCalibrationSession(direction_tolerance_mm=0.5)
    assert session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lower_authorized=True,
    ).lower_valve == 0x10

    command = session.step(
        now=0.05,
        sample=sample(0.05, 100.8),
        feedback=feedback(0.05),
        lower_authorized=True,
    )

    assert command == command.safe_stop()
    assert session.failed
    assert "方向反向" in (session.fault_reason or "")
    assert session.trials == ()
    assert session.step(
        now=0.85,
        sample=sample(0.85, 99.0),
        feedback=feedback(0.85),
        lower_authorized=True,
    ) == command.safe_stop()


def test_lower_session_missing_feedback_and_clock_rollback_fail_closed() -> None:
    missing = LowerCalibrationSession()
    assert missing.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=None,
        lower_authorized=True,
    ).lower_valve == 0
    assert missing.failed

    rollback = LowerCalibrationSession()
    assert rollback.step(
        now=0.1,
        sample=sample(0.1, 100.0),
        feedback=feedback(0.1),
        lower_authorized=True,
    ).lower_valve == 0x10
    assert rollback.step(
        now=0.05,
        sample=sample(0.05, 100.0),
        feedback=feedback(0.05),
        lower_authorized=True,
    ).lower_valve == 0
    assert rollback.failed
    assert "回退" in (rollback.fault_reason or "")


def test_calibration_store_round_trip_and_atomic_file(tmp_path) -> None:
    lift = analyze_lift_trials(complete_lift_trials())
    lower = analyze_lower_trials(complete_lower_trials()).confirm_comfortable(0x50)
    bundle = CalibrationBundle.from_results(lift, lower, soft_upper_limit_mm=2750.0)
    path = tmp_path / "state" / "calibration.json"
    store = CalibrationStore(path)

    store.save(bundle)

    assert store.load() == bundle
    assert json.loads(path.read_text(encoding="utf-8"))["schema_version"] == 1
    assert list(path.parent.glob("*.tmp")) == []


@pytest.mark.parametrize(
    "payload",
    [
        "{broken",
        json.dumps({"schema_version": 2}),
        json.dumps({"schema_version": 1, "unexpected": 1}),
        '{"schema_version":1,"min_stable_pwm":50,"coarse_pwm":70,'
        '"response_delay_s":NaN,"max_coast_mm":2,"peak_current_by_pwm":{},'
        '"lower_min_start_valve":48,"lower_comfortable_valve":80,'
        '"soft_upper_limit_mm":null}',
    ],
)
def test_calibration_store_rejects_bad_files(tmp_path, payload: str) -> None:
    path = tmp_path / "calibration.json"
    path.write_text(payload, encoding="utf-8")

    with pytest.raises(CalibrationError):
        CalibrationStore(path).load()


def test_calibration_store_rejects_boolean_schema_version(tmp_path) -> None:
    lift = analyze_lift_trials(complete_lift_trials())
    lower = analyze_lower_trials(complete_lower_trials()).confirm_comfortable(0x50)
    raw = CalibrationBundle.from_results(lift, lower).to_json_object()
    raw["schema_version"] = True
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="schema_version"):
        CalibrationStore(path).load()


@pytest.mark.parametrize(
    ("changes", "field"),
    [
        ({"min_stable_pwm": 41}, "min_stable_pwm"),
        ({"coarse_pwm": 71}, "coarse_pwm"),
        ({"response_delay_s": 0.301}, "response_delay_s"),
        ({"lower_min_start_valve": 0x31}, "lower_min_start_valve"),
        ({"lower_comfortable_valve": 0x51}, "lower_comfortable_valve"),
        ({"lower_comfortable_valve": 0x20}, "lower_comfortable_valve"),
    ],
)
def test_calibration_bundle_rejects_values_outside_measured_plan(
    changes: dict[str, object], field: str
) -> None:
    values = {
        "min_stable_pwm": 50,
        "coarse_pwm": 70,
        "response_delay_s": 0.15,
        "max_coast_mm": 5.0,
        "peak_current_by_pwm": {pwm: pwm * 10 for pwm in LIFT_PWM_LEVELS},
        "lower_min_start_valve": 0x30,
        "lower_comfortable_valve": 0x50,
        "soft_upper_limit_mm": None,
    }
    values.update(changes)

    with pytest.raises(CalibrationError, match=field):
        CalibrationBundle(**values)  # type: ignore[arg-type]


def test_calibration_store_rejects_noncanonical_pwm_keys(tmp_path) -> None:
    lift = analyze_lift_trials(complete_lift_trials())
    lower = analyze_lower_trials(complete_lower_trials()).confirm_comfortable(0x50)
    raw = CalibrationBundle.from_results(lift, lower).to_json_object()
    raw["peak_current_by_pwm"]["+40"] = raw["peak_current_by_pwm"].pop("40")
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="peak_current_by_pwm"):
        CalibrationStore(path).load()


def test_calibration_bundle_copies_and_freezes_peak_current_mapping() -> None:
    peaks = {pwm: pwm * 10 for pwm in LIFT_PWM_LEVELS}
    bundle = CalibrationBundle(
        min_stable_pwm=50,
        coarse_pwm=70,
        response_delay_s=0.15,
        max_coast_mm=5.0,
        peak_current_by_pwm=peaks,
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
    )
    peaks.clear()

    assert bundle.peak_current_by_pwm[50] == 500
    with pytest.raises(TypeError):
        bundle.peak_current_by_pwm[50] = 0  # type: ignore[index]


def test_calibration_bundle_exposes_peak_currents_as_read_only_mapping() -> None:
    assert get_type_hints(CalibrationBundle)["peak_current_by_pwm"] == Mapping[int, int]


def test_calibration_store_revalidates_bundle_before_writing(tmp_path) -> None:
    lift = analyze_lift_trials(complete_lift_trials())
    lower = analyze_lower_trials(complete_lower_trials()).confirm_comfortable(0x50)
    bundle = CalibrationBundle.from_results(lift, lower)
    object.__setattr__(bundle, "peak_current_by_pwm", {})
    path = tmp_path / "calibration.json"

    with pytest.raises(CalibrationError, match="peak_current_by_pwm"):
        CalibrationStore(path).save(bundle)
    assert not path.exists()
