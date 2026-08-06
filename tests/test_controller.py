import math

import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    ControlConfig,
    ControllerState,
    HeightController,
    HeightSample,
    PumpFeedback,
)


def control_config(**changes: float) -> ControlConfig:
    values = {
        "tolerance_mm": 2.0,
        "stable_time_s": 0.5,
        "overshoot_limit_mm": 5.0,
        "absolute_max_height_mm": 2900.0,
        "max_speed_mm_s": 1200.0,
        "sensor_timeout_s": 0.1,
        "control_loop_timeout_s": 0.1,
        "current_multiplier": 1.5,
        "current_duration_s": 0.2,
        "direction_tolerance_mm": 1.0,
        "survey_max_on_s": 1.0,
        "survey_pause_s": 0.5,
    }
    values.update(changes)
    return ControlConfig(**values)


def calibration(*, soft_limit: float | None = 2800.0, coast: float = 5.0) -> CalibrationBundle:
    return CalibrationBundle(
        min_stable_pwm=50,
        coarse_pwm=70,
        response_delay_s=0.15,
        max_coast_mm=coast,
        peak_current_by_pwm={pwm: pwm * 10 for pwm in range(40, 81, 5)},
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=soft_limit,
    )


def sample(now: float, height: float, *, timestamp: float | None = None, valid: bool = True):
    return HeightSample(
        now if timestamp is None else timestamp,
        100,
        height,
        valid,
        None if valid else "sensor error",
    )


def feedback(now: float, *, timestamp: float | None = None, current: int = 0, fault: int = 0):
    return PumpFeedback(now if timestamp is None else timestamp, current, fault, 0)


def step(
    controller: HeightController,
    now: float,
    height: float,
    *,
    lift_authorized: bool = True,
    lower_authorized: bool = False,
    current: int = 0,
):
    return controller.step(
        now=now,
        sample=sample(now, height),
        feedback=feedback(now, current=current),
        lift_authorized=lift_authorized,
        lower_authorized=lower_authorized,
    )


def test_control_zones_derive_from_coast_and_use_deterministic_boundaries() -> None:
    controller = HeightController(control_config(), calibration(coast=5.0))
    assert controller.slow_zone_mm == 50.0
    assert controller.pulse_zone_mm == 15.0
    controller.set_target(200.0)

    coarse = step(controller, 0.0, 100.0)
    assert controller.state is ControllerState.COARSE_LIFT
    assert coarse.lift_pwm == 70

    controller = HeightController(control_config(), calibration(coast=5.0))
    controller.set_target(150.0)
    boundary = step(controller, 0.0, 100.0)
    assert controller.state is ControllerState.P_CONTROL
    assert boundary.lift_pwm == 70

    controller = HeightController(control_config(), calibration(coast=5.0))
    controller.set_target(130.0)
    proportional = step(controller, 0.0, 100.0)
    assert controller.state is ControllerState.P_CONTROL
    assert 50 < proportional.lift_pwm < 70

    controller = HeightController(control_config(), calibration(coast=5.0))
    controller.set_target(115.0)
    pulse = step(controller, 0.0, 100.0)
    assert controller.state is ControllerState.TERMINAL_PULSE
    assert pulse.lift_pwm == 50


def test_terminal_pulse_uses_clamped_response_and_wait_phases() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(110.0)

    assert controller.pulse_on_s == pytest.approx(0.15)
    assert controller.pulse_wait_s == pytest.approx(0.65)
    assert step(controller, 0.0, 100.0).lift_pwm == 50
    assert step(controller, 0.1, 100.0).lift_pwm == 50
    assert step(controller, 0.15, 100.0).lift_pwm == 0
    for now in (0.25, 0.35, 0.45, 0.55, 0.65, 0.75):
        assert step(controller, now, 100.0).lift_pwm == 0
    assert step(controller, 0.8, 100.0).lift_pwm == 50


def test_target_requires_500ms_continuous_tolerance_before_hold() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    for now in (0.0, 0.1, 0.2, 0.3, 0.4):
        assert step(controller, now, 99.0).lift_pwm == 0
        assert controller.state is not ControllerState.HOLD
    assert step(controller, 0.5, 99.0).lift_pwm == 0
    assert controller.state is ControllerState.HOLD


def test_overshoot_never_auto_lowers_and_faults_only_beyond_limit() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    command = step(controller, 0.0, 103.0)
    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.IDLE
    assert controller.trial_failed
    assert controller.fault_reason is None

    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)
    command = step(controller, 0.0, 106.0)
    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.FAULT
    assert "超调" in (controller.fault_reason or "")


def test_lift_authorization_loss_is_immediate_zero_without_reverse_output() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    assert step(controller, 0.0, 100.0).lift_pwm == 70
    stopped = step(controller, 0.05, 100.0, lift_authorized=False, lower_authorized=True)

    assert stopped.lift_pwm == stopped.lower_valve == 0
    assert controller.state is ControllerState.COARSE_LIFT


def test_manual_lower_is_exclusive_and_boolean_authorized() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    controller.set_manual_lower(True)

    denied = step(controller, 0.0, 100.0, lower_authorized=False)
    assert denied.lift_pwm == denied.lower_valve == 0
    allowed = step(controller, 0.05, 100.0, lower_authorized=True)
    assert allowed.lift_pwm == 0
    assert allowed.lower_valve == 0x50
    assert controller.state is ControllerState.MANUAL_LOWER


def test_target_requires_effective_soft_limit_and_never_exceeds_absolute_limit() -> None:
    without_persistent_limit = HeightController(control_config(), calibration(soft_limit=None))
    with pytest.raises(ValueError, match="临时最大高度"):
        without_persistent_limit.set_target(1000.0)
    with pytest.raises(ValueError, match="2900"):
        without_persistent_limit.set_target(1000.0, temporary_max_height_mm=2901.0)
    without_persistent_limit.set_target(1000.0, temporary_max_height_mm=1200.0)

    with_persistent_limit = HeightController(control_config(), calibration(soft_limit=2800.0))
    with pytest.raises(ValueError, match="软限位"):
        with_persistent_limit.set_target(2800.1)


def test_current_height_above_effective_soft_limit_latches_fault() -> None:
    controller = HeightController(control_config(), calibration(soft_limit=2800.0))

    command = step(controller, 0.0, 2800.1)

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.FAULT
    assert "软限位" in (controller.fault_reason or "")


@pytest.mark.parametrize(
    ("now", "sample_value", "feedback_value", "reason"),
    [
        (0.0, HeightSample(0.0, 1, 100.0, False, "bad"), feedback(0.0), "无效"),
        (0.101, sample(0.101, 100.0, timestamp=0.0), feedback(0.101), "传感器"),
        (0.0, sample(0.0, 100.0), None, "CAN"),
        (0.151, sample(0.151, 100.0), feedback(0.151, timestamp=0.0), "CAN"),
        (0.0, sample(0.0, 100.0), feedback(0.0, fault=7), "故障码"),
        (0.0, HeightSample(0.0, None, 100.0, True, None), feedback(0.0), "raw"),
        (0.0, sample(0.0, math.inf), feedback(0.0), "高度"),
        (0.0, HeightSample(0.0, 1, 100.0, 1, None), feedback(0.0), "无效"),
    ],
)
def test_each_input_guard_fails_closed(
    now: float, sample_value, feedback_value, reason: str
) -> None:
    controller = HeightController(control_config(), calibration())

    command = controller.step(
        now=now,
        sample=sample_value,
        feedback=feedback_value,
        lift_authorized=True,
        lower_authorized=False,
    )

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.FAULT
    assert reason in (controller.fault_reason or "")


def test_control_loop_timeout_latches_fault() -> None:
    controller = HeightController(control_config(), calibration())
    step(controller, 0.0, 100.0)

    command = step(controller, 0.101, 100.0)

    assert command.lift_pwm == 0
    assert "控制循环" in (controller.fault_reason or "")


def test_adjacent_sample_speed_and_lift_direction_are_guarded() -> None:
    speed_controller = HeightController(control_config(), calibration())
    step(speed_controller, 0.0, 100.0)
    command = step(speed_controller, 0.05, 161.0)
    assert command.lift_pwm == 0
    assert "速度" in (speed_controller.fault_reason or "")

    direction_controller = HeightController(control_config(), calibration())
    direction_controller.set_target(300.0)
    step(direction_controller, 0.0, 100.0)
    command = step(direction_controller, 0.05, 98.9)
    assert command.lift_pwm == 0
    assert "方向" in (direction_controller.fault_reason or "")


def test_direction_fault_clear_requires_500ms_stable_observation_and_zero_clear_cycle() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    assert step(controller, 0.0, 100.0).lift_pwm == 70

    assert step(controller, 0.05, 98.9).lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert controller.fault_kind == "direction"
    assert controller.fault_height_mm == pytest.approx(98.9)
    assert controller.fault_timestamp == pytest.approx(0.05)

    controller.clear_fault()
    assert step(controller, 0.1, 98.0).lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    # 相对故障高度累计下降超过 1 mm，必须从新低点重新计稳定窗口。
    assert step(controller, 0.2, 97.8).lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    for now in (0.3, 0.4, 0.5, 0.6):
        assert step(controller, now, 97.8).lift_pwm == 0
        assert controller.state is ControllerState.FAULT

    cleared = step(controller, 0.7, 97.8)
    assert cleared.lift_pwm == cleared.lower_valve == 0
    assert controller.state is ControllerState.IDLE
    assert controller.fault_reason is None
    assert step(controller, 0.75, 97.8).lift_pwm == 70


@pytest.mark.parametrize(
    "mode",
    [
        ControllerState.LIFT_CALIBRATION,
        ControllerState.LOWER_CALIBRATION,
        ControllerState.SURVEY,
    ],
)
def test_external_modes_are_reachable_and_exclude_automatic_output(
    mode: ControllerState,
) -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    controller.enter_external_mode(mode)

    assert controller.state is mode
    assert controller.target_mm is None
    command = step(
        controller,
        0.0,
        100.0,
        lift_authorized=True,
        lower_authorized=True,
    )
    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is mode
    with pytest.raises(RuntimeError, match="外部模式"):
        controller.set_target(200.0)
    with pytest.raises(RuntimeError, match="外部模式"):
        controller.set_manual_lower(True)
    controller.cancel()
    assert controller.state is mode
    controller.exit_external_mode()
    assert controller.state is ControllerState.MONITOR


def test_external_mode_cancels_manual_lower_and_rejects_non_external_state() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_manual_lower(True)

    controller.enter_external_mode(ControllerState.LOWER_CALIBRATION)

    assert step(
        controller, 0.0, 100.0, lower_authorized=True
    ).lower_valve == 0
    with pytest.raises(ValueError, match="外部"):
        controller.enter_external_mode(ControllerState.HOLD)


def test_enter_external_mode_discards_old_lift_direction_and_current_history() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(600.0)
    assert step(controller, 0.0, 300.0, current=1100).lift_pwm == 70
    assert step(controller, 0.1, 300.0, current=1100).lift_pwm == 70
    assert step(controller, 0.2, 300.0, current=1100).lift_pwm == 70

    controller.enter_external_mode(ControllerState.SURVEY)
    command = step(controller, 0.3, 98.9, current=65535)

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.SURVEY
    assert controller.fault_reason is None


def test_same_pwm_overcurrent_must_persist_for_200ms() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    assert step(controller, 0.0, 100.0, current=1100).lift_pwm == 70
    assert step(controller, 0.1, 100.0, current=1100).lift_pwm == 70
    assert step(controller, 0.2, 100.0, current=1100).lift_pwm == 70
    command = step(controller, 0.3, 100.0, current=1100)

    assert command.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "电流" in (controller.fault_reason or "")

    controller.clear_fault()
    still_high = step(controller, 0.4, 100.0, current=65535)
    assert still_high.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "未恢复" in (controller.fault_reason or "")

    controller.clear_fault()
    recovered = step(controller, 0.5, 100.0, current=0)
    assert recovered.lift_pwm == 70
    assert controller.state is ControllerState.COARSE_LIFT


def test_p_control_quantizes_up_to_a_measured_pwm_level() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(130.0)

    command = step(controller, 0.0, 100.0)

    assert command.lift_pwm == 60
    assert command.lift_pwm in controller.calibration.peak_current_by_pwm


def test_quantized_p_pwm_has_its_own_200ms_overcurrent_guard() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(130.0)

    assert step(controller, 0.0, 100.0, current=1000).lift_pwm == 60
    assert step(controller, 0.1, 100.0, current=1000).lift_pwm == 60
    assert step(controller, 0.2, 100.0, current=1000).lift_pwm == 60
    command = step(controller, 0.3, 100.0, current=1000)

    assert command.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "PWM 60" in (controller.fault_reason or "")


def test_overcurrent_timer_restarts_when_actual_pwm_changes() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(120.0)

    assert step(controller, 0.0, 100.0, current=1000).lift_pwm == 55
    assert step(controller, 0.1, 100.0, current=1000).lift_pwm == 55
    controller.set_target(130.0)
    assert step(controller, 0.2, 100.0, current=1000).lift_pwm == 60
    assert step(controller, 0.3, 100.0, current=1000).lift_pwm == 60
    assert step(controller, 0.4, 100.0, current=1000).lift_pwm == 60
    command = step(controller, 0.5, 100.0, current=1000)

    assert command.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "PWM 60" in (controller.fault_reason or "")


def test_manual_lower_current_never_uses_lift_peak_guard() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_manual_lower(True)

    for index in range(5):
        command = step(
            controller,
            index * 0.1,
            100.0,
            lower_authorized=True,
            current=65535,
        )
        assert command.lower_valve == 0x50
        assert controller.state is ControllerState.MANUAL_LOWER


def test_feedback_timeout_cannot_be_configured_above_150ms() -> None:
    with pytest.raises(ValueError, match="0.15"):
        HeightController(control_config(), calibration(), feedback_timeout_s=0.1501)


@pytest.mark.parametrize(
    ("now", "sample_value", "feedback_value", "reason"),
    [
        (math.nan, sample(0.0, 100.0), feedback(0.0), "now"),
        (0.0, HeightSample(-1.0, 1, 100.0, True, None), feedback(0.0), "传感器时间戳"),
        (0.0, sample(0.0, 100.0), PumpFeedback(math.inf, 0, 0, 0), "CAN 反馈时间戳"),
    ],
)
def test_all_control_timestamps_must_be_finite_and_nonnegative(
    now: float, sample_value, feedback_value, reason: str
) -> None:
    controller = HeightController(control_config(), calibration())

    command = controller.step(
        now=now,
        sample=sample_value,
        feedback=feedback_value,
        lift_authorized=True,
        lower_authorized=False,
    )

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.FAULT
    assert reason in (controller.fault_reason or "")


def test_fault_requires_explicit_clear_and_recovered_conditions() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    controller.step(
        now=0.0,
        sample=sample(0.0, 100.0, valid=False),
        feedback=feedback(0.0),
        lift_authorized=True,
        lower_authorized=False,
    )
    assert controller.state is ControllerState.FAULT

    assert step(controller, 0.05, 100.0).lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    controller.clear_fault()
    assert step(controller, 0.1, 100.0).lift_pwm == 70
    assert controller.state is ControllerState.COARSE_LIFT
    assert controller.fault_reason is None
