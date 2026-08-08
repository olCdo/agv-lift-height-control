import math

import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    ControlConfig,
    ControllerState,
    HeightController,
    HeightSample,
    LowerCalibrationSession,
    PumpCommand,
    PumpFeedback,
    UpperLimitSurvey,
)
from agv_lift_height_control.calibration import LIFT_PULSE_S


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


def single_level_calibration() -> CalibrationBundle:
    return CalibrationBundle(
        min_stable_pwm=40,
        coarse_pwm=40,
        response_delay_s=0.075,
        max_coast_mm=5.0,
        peak_current_by_pwm={40: 1000},
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=2800.0,
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
    assert pulse == PumpCommand.safe_stop()


@pytest.mark.parametrize(
    ("target_mm", "expected_pwm"),
    [(200.0, 40), (150.0, 40), (115.0, 0)],
)
def test_single_level_calibration_never_commands_above_40(
    target_mm: float, expected_pwm: int
) -> None:
    controller = HeightController(control_config(), single_level_calibration())
    controller.set_target(target_mm)

    command = step(controller, 0.0, 100.0)

    assert command.lift_pwm == expected_pwm


def test_field_transition_from_continuous_lift_stops_before_terminal_pulse() -> None:
    controller = HeightController(control_config(), single_level_calibration())
    controller.set_target(80.0)

    moving = step(controller, 0.0, 46.337890625)
    assert controller.state is ControllerState.P_CONTROL
    assert moving == PumpCommand(interlock=True, lift_pwm=40)

    terminal_entry = step(controller, 0.08, 68.505859375)
    assert controller.state is ControllerState.TERMINAL_PULSE
    assert terminal_entry == PumpCommand.safe_stop()


def test_terminal_pulse_settles_before_first_on_and_waits_after_each_pulse() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(110.0)

    assert controller.pulse_on_s == pytest.approx(LIFT_PULSE_S)
    assert controller.pulse_wait_s == pytest.approx(0.65)
    for index in range(13):
        assert step(controller, index / 20, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.64, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.65, 100.0).lift_pwm == 50
    assert step(controller, 0.70, 100.0).lift_pwm == 50
    assert step(controller, 0.74, 100.0).lift_pwm == 50
    assert step(controller, 0.75, 100.0) == PumpCommand.safe_stop()
    for index in range(12):
        assert step(controller, 0.80 + index / 20, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.39, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.40, 100.0).lift_pwm == 50


@pytest.mark.parametrize(
    ("height_mm", "expected_on_s"),
    [
        (72.94921875, 0.1),
        (77.83203125, 0.05),
    ],
)
def test_terminal_pulse_halves_action_inside_measured_coast_distance(
    height_mm: float, expected_on_s: float
) -> None:
    field_calibration = CalibrationBundle(
        min_stable_pwm=40,
        coarse_pwm=40,
        response_delay_s=0.184041741999863,
        max_coast_mm=4.296875,
        peak_current_by_pwm={40: 1112},
        lower_min_start_valve=0x50,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=None,
    )
    controller = HeightController(control_config(), field_calibration)
    controller.set_target(80.0, temporary_max_height_mm=200.0)

    assert step(controller, 0.0, height_mm) == PumpCommand.safe_stop()
    for now in (0.08, 0.16, 0.24, 0.32, 0.40, 0.48, 0.56, 0.64):
        assert step(controller, now, height_mm) == PumpCommand.safe_stop()

    started_at = controller.pulse_wait_s
    assert step(controller, started_at, height_mm).lift_pwm == 40
    assert step(controller, started_at + expected_on_s - 0.001, height_mm).lift_pwm == 40
    assert step(controller, started_at + expected_on_s, height_mm) == PumpCommand.safe_stop()


def test_terminal_authorization_loss_restarts_with_full_settle() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(110.0)

    for index in range(13):
        assert step(controller, index / 20, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.65, 100.0).lift_pwm == 50
    assert step(controller, 0.70, 100.0, lift_authorized=False) == PumpCommand.safe_stop()
    for index in range(13):
        assert step(controller, 0.71 + index / 20, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.35, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.36, 100.0).lift_pwm == 50


def test_terminal_settle_cannot_be_bypassed_by_small_downward_drift() -> None:
    controller = HeightController(control_config(), calibration(coast=5.0))
    controller.set_target(115.0)

    assert step(controller, 0.0, 100.0) == PumpCommand.safe_stop()
    # 轻微回落让误差重新大于15 mm，也必须先完成已经开始的停泵观察。
    assert step(controller, 0.05, 99.9) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.TERMINAL_PULSE
    for index in range(2, 13):
        assert step(controller, index / 20, 99.9) == PumpCommand.safe_stop()
    assert step(controller, 0.64, 99.9) == PumpCommand.safe_stop()

    resumed = step(controller, 0.65, 99.9)
    assert controller.state is ControllerState.P_CONTROL
    assert resumed.lift_pwm > 0


def test_target_requires_500ms_continuous_tolerance_before_hold() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    for now in (0.0, 0.1, 0.2, 0.3, 0.4):
        command = step(controller, now, 99.0, lift_authorized=False)
        assert command == PumpCommand.hydraulic_hold()
        assert controller.state is not ControllerState.HOLD
    assert (
        step(controller, 0.5, 99.0, lift_authorized=False)
        == PumpCommand.hydraulic_hold()
    )
    assert controller.state is ControllerState.HOLD


def test_hydraulic_hold_fails_to_all_zero_on_sensor_fault() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)
    assert (
        step(controller, 0.0, 99.0, lift_authorized=False)
        == PumpCommand.hydraulic_hold()
    )

    command = controller.step(
        now=0.1,
        sample=HeightSample(0.1, 100, 99.0, False, "sensor error"),
        feedback=feedback(0.1),
        lift_authorized=False,
        lower_authorized=False,
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT


def test_overshoot_never_auto_lowers_and_faults_only_beyond_limit() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    command = step(controller, 0.0, 103.0)
    assert command.lift_pwm == command.lower_valve == 0
    assert command.interlock is False
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


def test_controller_accepts_signed_current_zero_offset() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    command = step(controller, 0.0, 100.0, current=-18)

    assert command.lift_pwm == 70
    assert controller.state is ControllerState.COARSE_LIFT
    assert controller.fault_reason is None


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


def test_lift_direction_guard_uses_cumulative_continuous_output_reference() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    assert step(controller, 0.0, 100.0).lift_pwm == 70
    assert step(controller, 0.05, 99.1).lift_pwm == 70
    command = step(controller, 0.1, 98.2)

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.FAULT
    assert controller.fault_kind == "direction"


def test_zero_actual_output_resets_lift_direction_reference() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    assert step(controller, 0.0, 100.0).lift_pwm == 70
    assert step(controller, 0.05, 99.5, lift_authorized=False).lift_pwm == 0
    assert step(controller, 0.1, 98.6).lift_pwm == 70
    assert step(controller, 0.15, 97.7).lift_pwm == 70
    assert step(controller, 0.2, 96.8).lift_pwm == 0
    assert controller.fault_kind == "direction"


def test_lower_direction_guard_uses_cumulative_continuous_output_reference() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(ControllerState.LOWER_CALIBRATION)
    desired = PumpCommand(interlock=True, lower_valve=0x10)

    for now, height in ((0.0, 100.0), (0.05, 100.6)):
        assert controller.step_external(
            now=now,
            sample=sample(now, height),
            feedback=feedback(now),
            desired_command=desired,
            lift_authorized=False,
            lower_authorized=True,
        ).lower_valve == 0x10
    command = controller.step_external(
        now=0.1,
        sample=sample(0.1, 101.2),
        feedback=feedback(0.1),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT
    assert controller.fault_kind == "lower_direction"


def test_zero_actual_output_resets_lower_direction_reference() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(ControllerState.LOWER_CALIBRATION)
    desired = PumpCommand(interlock=True, lower_valve=0x10)

    assert controller.step_external(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ).lower_valve == 0x10
    assert controller.step_external(
        now=0.05,
        sample=sample(0.05, 100.5),
        feedback=feedback(0.05),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=False,
    ) == PumpCommand.safe_stop()
    for now, height in ((0.1, 101.1), (0.15, 101.7)):
        assert controller.step_external(
            now=now,
            sample=sample(now, height),
            feedback=feedback(now),
            desired_command=desired,
            lift_authorized=False,
            lower_authorized=True,
        ).lower_valve == 0x10
    assert controller.step_external(
        now=0.2,
        sample=sample(0.2, 102.3),
        feedback=feedback(0.2),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ) == PumpCommand.safe_stop()
    assert controller.fault_kind == "lower_direction"


def test_lower_direction_fault_clear_requires_stable_window_and_zero_cycle() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(ControllerState.LOWER_CALIBRATION)
    desired = PumpCommand(interlock=True, lower_valve=0x10)
    for now, height in ((0.0, 100.0), (0.05, 101.2)):
        controller.step_external(
            now=now,
            sample=sample(now, height),
            feedback=feedback(now),
            desired_command=desired,
            lift_authorized=False,
            lower_authorized=True,
        )
    assert controller.fault_kind == "lower_direction"

    controller.clear_fault()
    assert controller.step_external(
        now=0.1,
        sample=sample(0.1, 102.0),
        feedback=feedback(0.1),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ) == PumpCommand.safe_stop()
    assert controller.step_external(
        now=0.2,
        sample=sample(0.2, 102.3),
        feedback=feedback(0.2),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ) == PumpCommand.safe_stop()
    for now in (0.3, 0.4, 0.5, 0.6):
        assert controller.step_external(
            now=now,
            sample=sample(now, 102.3),
            feedback=feedback(now),
            desired_command=desired,
            lift_authorized=False,
            lower_authorized=True,
        ) == PumpCommand.safe_stop()
        assert controller.state is ControllerState.FAULT
    assert controller.step_external(
        now=0.7,
        sample=sample(0.7, 102.3),
        feedback=feedback(0.7),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.LOWER_CALIBRATION
    assert controller.step_external(
        now=0.75,
        sample=sample(0.75, 102.3),
        feedback=feedback(0.75),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    ).lower_valve == 0x10


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


def test_controller_state_defines_emergency_stop() -> None:
    assert ControllerState.EMERGENCY_STOP.value == "emergency_stop"


@pytest.mark.parametrize("reason", [None, 123, "", "   "])
def test_emergency_stop_reason_must_be_a_nonblank_string(reason: object) -> None:
    controller = HeightController(control_config(), calibration())

    with pytest.raises((TypeError, ValueError)):
        controller.enter_emergency_stop(reason)  # type: ignore[arg-type]


def test_emergency_stop_cancels_target_and_blocks_all_normal_motion_requests() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    assert step(controller, 0.0, 100.0).lift_pwm == 70

    controller.enter_emergency_stop("急停按钮按下")

    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.emergency_stop_reason == "急停按钮按下"
    assert controller.target_mm is None
    assert step(controller, 0.05, 100.0) == PumpCommand.safe_stop()
    with pytest.raises(RuntimeError, match="急停"):
        controller.set_target(200.0)
    with pytest.raises(RuntimeError, match="急停"):
        controller.set_manual_lower(True)
    with pytest.raises(RuntimeError, match="急停"):
        controller.enter_external_mode(ControllerState.SURVEY)

    controller.enter_emergency_stop("后续原因不得覆盖首次原因")
    controller.cancel()
    controller.clear_fault()
    assert controller.emergency_stop_reason == "急停按钮按下"
    assert step(controller, 0.1, 100.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.EMERGENCY_STOP


def test_emergency_stop_overrides_normal_step_before_any_input_validation() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_emergency_stop("普通控制周期急停")

    command = controller.step(
        now=math.nan,
        sample=None,  # type: ignore[arg-type]
        feedback="bad feedback",  # type: ignore[arg-type]
        lift_authorized="yes",  # type: ignore[arg-type]
        lower_authorized=None,  # type: ignore[arg-type]
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.emergency_stop_reason == "普通控制周期急停"


def test_emergency_stop_overrides_external_mode_before_any_input_validation() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(ControllerState.SURVEY)
    controller.enter_emergency_stop("外部急停输入")

    command = controller.step_external(
        now=math.nan,
        sample=None,  # type: ignore[arg-type]
        feedback="bad feedback",  # type: ignore[arg-type]
        desired_command=object(),  # type: ignore[arg-type]
        lift_authorized="yes",  # type: ignore[arg-type]
        lower_authorized=None,  # type: ignore[arg-type]
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.emergency_stop_reason == "外部急停输入"


def test_exit_emergency_stop_never_restores_old_target_or_manual_lower() -> None:
    target_controller = HeightController(control_config(), calibration())
    target_controller.set_target(110.0)
    assert step(target_controller, 0.0, 100.0) == PumpCommand.safe_stop()
    assert target_controller.state is ControllerState.TERMINAL_PULSE

    target_controller.enter_emergency_stop("现场复位前急停")
    target_controller.exit_emergency_stop()

    assert target_controller.state is ControllerState.MONITOR
    assert target_controller.emergency_stop_reason is None
    assert target_controller.target_mm is None
    assert step(target_controller, 0.05, 100.0) == PumpCommand.safe_stop()

    manual_controller = HeightController(control_config(), calibration())
    manual_controller.set_manual_lower(True)
    manual_controller.enter_emergency_stop("人工下降时急停")
    manual_controller.exit_emergency_stop()
    assert step(
        manual_controller,
        0.0,
        100.0,
        lower_authorized=True,
    ) == PumpCommand.safe_stop()
    manual_controller.exit_emergency_stop()
    assert manual_controller.state is ControllerState.MONITOR


def test_exit_emergency_stop_restores_preexisting_fault_until_explicit_clear() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    fault_command = controller.step(
        now=0.0,
        sample=sample(0.0, 100.0, valid=False),
        feedback=feedback(0.0),
        lift_authorized=True,
        lower_authorized=False,
    )
    assert fault_command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT

    controller.enter_emergency_stop("故障期间触发急停")
    controller.exit_emergency_stop()

    assert controller.state is ControllerState.FAULT
    assert controller.target_mm is None
    assert step(controller, 0.05, 100.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT

    controller.clear_fault()
    assert step(controller, 0.1, 100.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.MONITOR


def test_exit_emergency_stop_resets_control_cycle_timeout_baseline() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)
    assert step(controller, 0.0, 100.0).lift_pwm == 70

    controller.enter_emergency_stop("长时间急停")
    controller.exit_emergency_stop()
    first_cycle = step(controller, 10.0, 100.0)

    assert first_cycle == PumpCommand.safe_stop()
    assert controller.state is ControllerState.MONITOR
    assert controller.fault_reason is None


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
    command = step(controller, 0.3, 98.9, current=32767)

    assert command.lift_pwm == command.lower_valve == 0
    assert controller.state is ControllerState.SURVEY
    assert controller.fault_reason is None


def test_external_survey_commands_share_direction_overcurrent_and_feedback_guards() -> None:
    direction = HeightController(control_config(), calibration())
    direction.enter_external_mode(ControllerState.SURVEY)
    survey = UpperLimitSurvey(
        control_config(), calibration(), temporary_max_height_mm=1200.0
    )
    for now, height in ((0.0, 100.0), (0.05, 99.1)):
        desired = survey.step(
            now=now, sample=sample(now, height), lift_authorized=True
        )
        assert direction.step_external(
            now=now,
            sample=sample(now, height),
            feedback=feedback(now),
            desired_command=desired,
            lift_authorized=True,
            lower_authorized=False,
        ).lift_pwm == 50
    desired = survey.step(
        now=0.1, sample=sample(0.1, 98.2), lift_authorized=True
    )
    reverse = direction.step_external(
        now=0.1,
        sample=sample(0.1, 98.2),
        feedback=feedback(0.1),
        desired_command=desired,
        lift_authorized=True,
        lower_authorized=False,
    )
    assert reverse == PumpCommand.safe_stop()
    assert direction.fault_kind == "direction"

    overcurrent = HeightController(control_config(), calibration())
    overcurrent.enter_external_mode(ControllerState.SURVEY)
    desired_lift = PumpCommand(interlock=True, lift_pwm=50)
    for now in (0.0, 0.1, 0.2):
        assert overcurrent.step_external(
            now=now,
            sample=sample(now, 100.0),
            feedback=feedback(now, current=800),
            desired_command=desired_lift,
            lift_authorized=True,
            lower_authorized=False,
        ).lift_pwm == 50
    assert overcurrent.step_external(
        now=0.3,
        sample=sample(0.3, 100.0),
        feedback=feedback(0.3, current=800),
        desired_command=desired_lift,
        lift_authorized=True,
        lower_authorized=False,
    ) == PumpCommand.safe_stop()
    assert overcurrent.fault_kind == "overcurrent"

    feedback_fault = HeightController(control_config(), calibration())
    feedback_fault.enter_external_mode(ControllerState.SURVEY)
    assert feedback_fault.step_external(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0, fault=7),
        desired_command=desired_lift,
        lift_authorized=True,
        lower_authorized=False,
    ) == PumpCommand.safe_stop()
    assert feedback_fault.state is ControllerState.FAULT
    assert feedback_fault.step_external(
        now=0.05,
        sample=sample(0.05, 100.0),
        feedback=feedback(0.05),
        desired_command=desired_lift,
        lift_authorized=True,
        lower_authorized=False,
    ) == PumpCommand.safe_stop()


def test_external_lower_session_command_requires_lower_authorization() -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(ControllerState.LOWER_CALIBRATION)
    session = LowerCalibrationSession()
    desired = session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lower_authorized=True,
    )

    denied = controller.step_external(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=False,
    )
    allowed = controller.step_external(
        now=0.05,
        sample=sample(0.05, 100.0),
        feedback=feedback(0.05),
        desired_command=desired,
        lift_authorized=False,
        lower_authorized=True,
    )

    assert denied == PumpCommand.safe_stop()
    assert allowed.lift_pwm == 0
    assert allowed.lower_valve == 0x10


def test_step_external_requires_external_mode_and_exclusive_command() -> None:
    controller = HeightController(control_config(), calibration())
    with pytest.raises(RuntimeError, match="外部模式"):
        controller.step_external(
            now=0.0,
            sample=sample(0.0, 100.0),
            feedback=feedback(0.0),
            desired_command=PumpCommand.safe_stop(),
            lift_authorized=True,
            lower_authorized=True,
        )

    controller.enter_external_mode(ControllerState.SURVEY)
    command = controller.step_external(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        desired_command=PumpCommand(interlock=True, lift_pwm=50, lower_valve=0x10),
        lift_authorized=True,
        lower_authorized=True,
    )
    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT


@pytest.mark.parametrize(
    ("mode", "desired"),
    [
        (ControllerState.LIFT_CALIBRATION, PumpCommand(interlock=True, lift_pwm=73)),
        (ControllerState.SURVEY, PumpCommand(interlock=True, lift_pwm=55)),
        (
            ControllerState.LOWER_CALIBRATION,
            PumpCommand(interlock=True, lower_valve=0x11),
        ),
    ],
)
def test_step_external_rejects_commands_outside_mode_measurement_plan(
    mode: ControllerState, desired: PumpCommand
) -> None:
    controller = HeightController(control_config(), calibration())
    controller.enter_external_mode(mode)

    command = controller.step_external(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0, current=32767),
        desired_command=desired,
        lift_authorized=True,
        lower_authorized=True,
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT
    assert "实测计划" in (controller.fault_reason or "")


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
    still_high = step(controller, 0.4, 100.0, current=32767)
    assert still_high.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "未恢复" in (controller.fault_reason or "")

    controller.clear_fault()
    recovered = step(controller, 0.5, 100.0, current=0)
    assert recovered.lift_pwm == 70
    assert controller.state is ControllerState.COARSE_LIFT


def test_negative_polarity_overcurrent_uses_current_magnitude() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(300.0)

    assert step(controller, 0.0, 100.0, current=-1100).lift_pwm == 70
    assert step(controller, 0.1, 100.0, current=-1100).lift_pwm == 70
    assert step(controller, 0.2, 100.0, current=-1100).lift_pwm == 70
    command = step(controller, 0.3, 100.0, current=-1100)

    assert command.lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    assert "电流" in (controller.fault_reason or "")


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
            current=32767,
        )
        assert command.lower_valve == 0x50
        assert controller.state is ControllerState.MANUAL_LOWER


@pytest.mark.parametrize("height", [2800.1, 2900.1])
def test_upper_limits_allow_authorized_manual_lower_recovery(height: float) -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_manual_lower(True)

    denied = step(controller, 0.0, height, lower_authorized=False)
    allowed = step(controller, 0.05, height, lower_authorized=True)

    assert denied.lift_pwm == denied.lower_valve == 0
    assert allowed.lift_pwm == 0
    assert allowed.lower_valve == 0x50
    assert controller.state is ControllerState.MANUAL_LOWER
    assert controller.fault_reason is None


def test_limit_fault_clear_for_manual_lower_keeps_clear_cycle_zero() -> None:
    controller = HeightController(control_config(), calibration())

    assert step(controller, 0.0, 2800.1).lift_pwm == 0
    assert controller.state is ControllerState.FAULT
    controller.set_manual_lower(True)
    controller.clear_fault()

    clear_cycle = step(controller, 0.05, 2800.1, lower_authorized=True)
    assert clear_cycle.lift_pwm == clear_cycle.lower_valve == 0
    assert controller.state is ControllerState.MANUAL_LOWER
    assert step(controller, 0.1, 2800.1, lower_authorized=True).lower_valve == 0x50


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
