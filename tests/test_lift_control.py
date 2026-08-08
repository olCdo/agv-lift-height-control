"""公共定高门面的安全边界测试。"""

import threading

import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    ControlConfig,
    ControllerState,
    EmergencyStopLatch,
    HeightController,
    HeightSample,
    PumpCommand,
    PumpFeedback,
)


def test_package_root_exports_lift_height_control() -> None:
    from agv_lift_height_control import LiftHeightControl

    assert LiftHeightControl.__name__ == "LiftHeightControl"


def _controller() -> HeightController:
    return HeightController(
        ControlConfig(
            tolerance_mm=2.0,
            stable_time_s=0.5,
            overshoot_limit_mm=5.0,
            absolute_max_height_mm=2900.0,
            max_speed_mm_s=1200.0,
            sensor_timeout_s=0.1,
            control_loop_timeout_s=0.1,
            current_multiplier=1.5,
            current_duration_s=0.2,
            direction_tolerance_mm=1.0,
            survey_max_on_s=1.0,
            survey_pause_s=0.5,
        ),
        CalibrationBundle(
            min_stable_pwm=50,
            coarse_pwm=70,
            response_delay_s=0.15,
            max_coast_mm=5.0,
            peak_current_by_pwm={pwm: pwm * 10 for pwm in range(40, 81, 5)},
            lower_min_start_valve=0x30,
            lower_comfortable_valve=0x50,
            soft_upper_limit_mm=2800.0,
        ),
    )


def _sample(timestamp: float, height_mm: float | None = 100.0, *, valid=True):
    return HeightSample(timestamp, 100, height_mm, valid, None if valid else "坏样本")


def _feedback(timestamp: float, *, fault_code: int = 0):
    return PumpFeedback(timestamp, 0, fault_code, 0)


def _control(now: list[float]):
    from agv_lift_height_control import LiftHeightControl

    latch = EmergencyStopLatch(clock=lambda: now[0])
    controller = _controller()
    return LiftHeightControl(controller, latch, clock=lambda: now[0]), controller, latch


def test_update_runs_bidirectional_automatic_control_without_keyboard_authorization() -> None:
    now = [0.0]
    control, controller, _latch = _control(now)
    control.set_target_height(300.0)

    lift = control.update(0.0, _sample(0.0, 100.0), _feedback(0.0))

    assert lift.lift_pwm == 70
    lower_control, _lower_controller, _lower_latch = _control(now)
    lower_control.set_target_height(100.0)
    lower = lower_control.update(0.0, _sample(0.0, 300.0), _feedback(0.0))
    assert lower.lower_valve > 0


def test_emergency_stop_latches_first_reason_before_controller_and_blocks_targets() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.set_target_height(300.0)

    control.emergency_stop("现场按钮")
    control.emergency_stop("后续诊断")

    assert latch.snapshot().active is True
    assert latch.snapshot().reason == "现场按钮"
    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.emergency_stop_reason == "现场按钮"
    assert controller.target_mm is None
    with pytest.raises(RuntimeError, match="急停"):
        control.set_target_height(200.0)


def test_update_imports_a_pump_side_emergency_stop_into_the_controller() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    latch.trigger("CAN线程急停")

    command = control.update(1.0, _sample(1.0), _feedback(1.0))

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.emergency_stop_reason == "CAN线程急停"


def test_healthy_clear_does_not_restore_the_old_target_or_output() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.set_target_height(300.0)
    control.emergency_stop("复位测试")
    assert control.update(1.0, _sample(1.0), _feedback(1.0)) == PumpCommand.safe_stop()
    latch.record_send_success(PumpCommand.safe_stop())

    control.clear_emergency_stop()

    assert latch.snapshot().active is False
    assert controller.state is ControllerState.MONITOR
    assert controller.target_mm is None
    assert control.update(1.0, _sample(1.0), _feedback(1.0)) == PumpCommand.safe_stop()


def test_clear_synchronizes_a_pump_side_latch_before_releasing_it() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.set_target_height(300.0)
    assert control.update(1.0, _sample(1.0), _feedback(1.0)).lift_pwm > 0
    latch.trigger("泵侧急停")
    latch.record_send_success(PumpCommand.safe_stop())

    control.clear_emergency_stop()

    assert latch.snapshot().active is False
    assert controller.state is ControllerState.MONITOR
    assert controller.target_mm is None
    assert control.update(1.0, _sample(1.0), _feedback(1.0)) == PumpCommand.safe_stop()


def test_emergency_stop_retriggers_if_clear_wins_before_facade_lock() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    triggered = threading.Event()
    original_trigger = latch.trigger

    def observed_trigger(reason: str) -> None:
        original_trigger(reason)
        triggered.set()

    latch.trigger = observed_trigger  # type: ignore[method-assign]
    errors = []

    def stop_from_other_thread() -> None:
        try:
            control.emergency_stop("并发急停")
        except BaseException as exc:
            errors.append(exc)

    control._lock.acquire()
    worker = threading.Thread(target=stop_from_other_thread)
    try:
        worker.start()
        assert triggered.wait(timeout=1.0)
        latch.record_send_success(PumpCommand.safe_stop())
        latch.clear()
    finally:
        control._lock.release()
    worker.join(timeout=1.0)

    assert errors == []
    assert worker.is_alive() is False
    assert latch.snapshot().active is True
    assert latch.snapshot().reason == "并发急停"
    assert controller.state is ControllerState.EMERGENCY_STOP


@pytest.mark.parametrize(
    ("sample", "feedback", "message"),
    [
        (_sample(0.89), _feedback(1.0), "样本.*超时"),
        (_sample(1.01), _feedback(1.0), "样本.*未来"),
        (_sample(1.0), _feedback(0.849), "反馈.*超时"),
        (_sample(1.0), _feedback(1.01), "反馈.*未来"),
        (_sample(1.0, valid=False), _feedback(1.0), "样本.*无效"),
        (_sample(1.0, None), _feedback(1.0), "高度"),
        (_sample(1.0), _feedback(1.0, fault_code=7), "故障码"),
    ],
)
def test_clear_rejects_unhealthy_cached_inputs_without_partially_releasing(
    sample: HeightSample,
    feedback: PumpFeedback,
    message: str,
) -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.emergency_stop("健康门禁")
    control.update(1.0, sample, feedback)
    latch.record_send_success(PumpCommand.safe_stop())

    with pytest.raises(RuntimeError, match=message):
        control.clear_emergency_stop()

    assert latch.snapshot().active is True
    assert controller.state is ControllerState.EMERGENCY_STOP


def test_clear_requires_both_cached_sample_and_feedback() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.emergency_stop("缺少观测")
    latch.record_send_success(PumpCommand.safe_stop())

    with pytest.raises(RuntimeError, match="样本"):
        control.clear_emergency_stop()

    assert latch.snapshot().active is True
    assert controller.state is ControllerState.EMERGENCY_STOP


def test_clear_cannot_bypass_a_controller_fault_hidden_by_emergency_stop() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.set_target_height(300.0)
    control.update(0.9, _sample(0.9, valid=False), _feedback(0.9))
    assert controller.state is ControllerState.FAULT
    control.emergency_stop("故障后急停")
    control.update(1.0, _sample(1.0), _feedback(1.0))
    latch.record_send_success(PumpCommand.safe_stop())

    with pytest.raises(RuntimeError, match="普通故障"):
        control.clear_emergency_stop()

    assert latch.snapshot().active is True
    assert controller.state is ControllerState.EMERGENCY_STOP


def test_clear_requires_zero_send_evidence_and_a_recovered_transport() -> None:
    now = [1.0]
    control, controller, latch = _control(now)
    control.emergency_stop("发送证据")
    control.update(1.0, _sample(1.0), _feedback(1.0))

    with pytest.raises(RuntimeError, match="全零"):
        control.clear_emergency_stop()
    assert latch.snapshot().active is True
    assert controller.state is ControllerState.EMERGENCY_STOP

    latch.record_send_success(PumpCommand.safe_stop())
    latch.record_transport_fault("CAN断线")
    with pytest.raises(RuntimeError, match="传输"):
        control.clear_emergency_stop()
    assert latch.snapshot().active is True
    assert controller.state is ControllerState.EMERGENCY_STOP
