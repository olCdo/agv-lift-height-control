import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    ControlConfig,
    ControllerState,
    HeightController,
    HydraulicLiftSimulator,
    PumpCommand,
)


def control_config() -> ControlConfig:
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
        survey_max_on_s=1,
        survey_pause_s=0.5,
    )


def calibration(*, coast: float, response: float) -> CalibrationBundle:
    return CalibrationBundle(
        min_stable_pwm=50,
        coarse_pwm=70,
        response_delay_s=response,
        max_coast_mm=coast,
        peak_current_by_pwm={pwm: pwm * 10 for pwm in range(40, 81, 5)},
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=2800,
    )


def test_simulator_models_dead_zone_response_delay_and_decaying_coast() -> None:
    simulator = HydraulicLiftSimulator(
        initial_height_mm=100,
        min_lift_pwm=45,
        response_delay_s=0.1,
        max_lift_speed_mm_s=300,
        coast_decay_s=0.1,
        fixed_step_s=0.05,
    )

    for _ in range(4):
        simulator.advance(PumpCommand(interlock=True, lift_pwm=40))
    assert simulator.height_mm == 100

    simulator.advance(PumpCommand(interlock=True, lift_pwm=70))
    before_response = simulator.advance(PumpCommand(interlock=True, lift_pwm=70)).height_mm
    assert before_response == 100
    moving = simulator.advance(PumpCommand(interlock=True, lift_pwm=70))
    assert moving.height_mm > 100

    first_stop = simulator.advance(PumpCommand.safe_stop())
    second_stop = simulator.advance(PumpCommand.safe_stop())
    third_stop = simulator.advance(PumpCommand.safe_stop())
    assert first_stop.height_mm > moving.height_mm
    assert 0 < third_stop.height_mm - second_stop.height_mm < second_stop.height_mm - first_stop.height_mm


def test_simulator_rejects_simultaneous_lift_and_lower() -> None:
    simulator = HydraulicLiftSimulator()

    with pytest.raises(ValueError, match="同时"):
        simulator.advance(PumpCommand(interlock=True, lift_pwm=50, lower_valve=0x50))


@pytest.mark.parametrize(
    ("initial", "target", "dead_zone", "response", "speed", "decay", "coast"),
    [
        (0.0, 120.0, 45, 0.05, 240.0, 0.04, 4.0),
        (300.0, 650.0, 42, 0.08, 280.0, 0.05, 6.0),
        (900.0, 1030.0, 48, 0.04, 220.0, 0.03, 3.0),
    ],
)
def test_controller_reaches_three_targets_in_deterministic_hydraulic_simulation(
    initial: float,
    target: float,
    dead_zone: int,
    response: float,
    speed: float,
    decay: float,
    coast: float,
) -> None:
    simulator = HydraulicLiftSimulator(
        initial_height_mm=initial,
        min_lift_pwm=dead_zone,
        response_delay_s=response,
        max_lift_speed_mm_s=speed,
        coast_decay_s=decay,
        fixed_step_s=0.05,
    )
    controller = HeightController(
        control_config(), calibration(coast=coast, response=max(response, 0.1))
    )
    controller.set_target(target)
    maximum = initial

    for _ in range(1200):
        snapshot = simulator.observe()
        command = controller.step(
            now=snapshot.now,
            sample=snapshot.sample,
            feedback=snapshot.feedback,
            lift_authorized=True,
            lower_authorized=False,
        )
        snapshot = simulator.advance(command)
        maximum = max(maximum, snapshot.height_mm)
        if controller.state is ControllerState.HOLD:
            break

    assert controller.state is ControllerState.HOLD, controller.fault_reason
    assert abs(simulator.height_mm - target) <= 2.0
    assert maximum - target <= 5.0
