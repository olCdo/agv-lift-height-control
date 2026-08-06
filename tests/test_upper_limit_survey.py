import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    CalibrationError,
    CalibrationStore,
    ControlConfig,
    HeightSample,
    UpperLimitSurvey,
)


def config() -> ControlConfig:
    return ControlConfig(
        tolerance_mm=2,
        stable_time_s=0.5,
        overshoot_limit_mm=5,
        absolute_max_height_mm=2900,
        max_speed_mm_s=1200,
        sensor_timeout_s=0.1,
        control_loop_timeout_s=0.1,
        current_multiplier=1.5,
        current_duration_s=0.2,
        direction_tolerance_mm=1,
        survey_max_on_s=1.0,
        survey_pause_s=0.5,
    )


def bundle(*, coast: float = 5.0) -> CalibrationBundle:
    return CalibrationBundle(
        min_stable_pwm=50,
        coarse_pwm=70,
        response_delay_s=0.15,
        max_coast_mm=coast,
        peak_current_by_pwm={pwm: pwm * 10 for pwm in range(40, 81, 5)},
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=None,
    )


def sample(now: float, height: float) -> HeightSample:
    return HeightSample(now, 100, height, True, None)


def test_survey_requires_valid_temporary_limit() -> None:
    with pytest.raises(CalibrationError, match="临时"):
        UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=None)
    with pytest.raises(CalibrationError, match="2900"):
        UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=2900.1)


def test_survey_limits_each_lift_segment_to_one_second_then_pauses() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)

    assert survey.step(now=0.0, sample=sample(0.0, 100.0), lift_authorized=True).lift_pwm == 50
    assert survey.step(now=0.999, sample=sample(0.999, 101.0), lift_authorized=True).lift_pwm == 50
    assert survey.step(now=1.0, sample=sample(1.0, 102.0), lift_authorized=True).lift_pwm == 0
    assert survey.step(now=1.499, sample=sample(1.499, 102.5), lift_authorized=True).lift_pwm == 0
    assert survey.step(now=1.5, sample=sample(1.5, 102.5), lift_authorized=True).lift_pwm == 50


def test_survey_authorization_loss_stops_and_resets_continuous_segment() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)
    survey.step(now=0.0, sample=sample(0.0, 100.0), lift_authorized=True)

    assert survey.step(now=0.2, sample=sample(0.2, 101.0), lift_authorized=False).lift_pwm == 0
    assert survey.step(now=0.3, sample=sample(0.3, 101.0), lift_authorized=True).lift_pwm == 50
    assert survey.step(now=1.299, sample=sample(1.299, 102.0), lift_authorized=True).lift_pwm == 50
    assert survey.step(now=1.3, sample=sample(1.3, 103.0), lift_authorized=True).lift_pwm == 0


def test_survey_authorization_loss_returns_zero_even_with_invalid_sample() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)
    survey.step(now=0.0, sample=sample(0.0, 100.0), lift_authorized=True)

    command = survey.step(
        now=0.1,
        sample=HeightSample(0.1, None, None, False, "lost"),
        lift_authorized=False,
    )

    assert command.lift_pwm == command.lower_valve == 0


def test_survey_invalid_sample_after_lift_fails_closed_without_exception() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)
    assert survey.step(
        now=0.0, sample=sample(0.0, 100.0), lift_authorized=True
    ).lift_pwm == 50

    command = survey.step(
        now=0.1,
        sample=HeightSample(0.1, None, None, False, "sensor lost"),
        lift_authorized=True,
    )

    assert command.lift_pwm == command.lower_valve == 0
    assert "有效高度样本" in (survey.fault_reason or "")
    assert survey.failed
    assert survey.step(
        now=0.2, sample=sample(0.2, 100.0), lift_authorized=True
    ).lift_pwm == 0
    with pytest.raises(CalibrationError, match="失败"):
        survey.confirm(bundle())


def test_survey_expired_sample_latches_failed_end_state() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)
    survey.step(now=0.0, sample=sample(0.0, 100.0), lift_authorized=True)

    command = survey.step(
        now=0.2,
        sample=sample(0.0, 101.0),
        lift_authorized=True,
    )

    assert command.lift_pwm == 0
    assert survey.failed
    assert "超时" in (survey.fault_reason or "")


def test_survey_authorization_toggle_cannot_bypass_mandatory_pause() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)
    survey.step(now=0.0, sample=sample(0.0, 100.0), lift_authorized=True)
    assert survey.step(
        now=1.0, sample=sample(1.0, 102.0), lift_authorized=True
    ).lift_pwm == 0

    assert survey.step(
        now=1.1,
        sample=HeightSample(1.1, None, None, False, "authorization lost"),
        lift_authorized=False,
    ).lift_pwm == 0
    assert survey.step(
        now=1.2, sample=sample(1.2, 102.0), lift_authorized=True
    ).lift_pwm == 0
    assert survey.step(
        now=1.5, sample=sample(1.5, 102.0), lift_authorized=True
    ).lift_pwm == 50


def test_survey_never_lifts_at_temporary_or_absolute_limit() -> None:
    survey = UpperLimitSurvey(config(), bundle(), temporary_max_height_mm=1200.0)

    command = survey.step(now=0.0, sample=sample(0.0, 1200.0), lift_authorized=True)

    assert command.lift_pwm == command.lower_valve == 0
    assert survey.limit_reached
    assert survey.highest_observed_mm == 1200.0
    assert survey.step(
        now=0.1, sample=sample(0.1, 1199.0), lift_authorized=True
    ).lift_pwm == 0


def test_survey_suggestion_formula_and_explicit_confirm_store(tmp_path) -> None:
    original = bundle(coast=30.0)
    survey = UpperLimitSurvey(config(), original, temporary_max_height_mm=1200.0)
    survey.step(now=0.0, sample=sample(0.0, 1000.0), lift_authorized=True)
    path = tmp_path / "calibration.json"
    store = CalibrationStore(path)

    assert survey.suggested_soft_limit_mm == 940.0
    assert original.soft_upper_limit_mm is None
    assert not path.exists()

    updated = survey.confirm(original, store=store)

    assert updated.soft_upper_limit_mm == 940.0
    assert store.load() == updated
