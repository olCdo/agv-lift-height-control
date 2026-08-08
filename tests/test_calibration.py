import json
from collections.abc import Mapping
from dataclasses import replace
from typing import get_type_hints

import pytest

from agv_lift_height_control import HeightSample, PumpCommand, PumpFeedback
from agv_lift_height_control.calibration import (
    LIFT_PWM_LEVELS,
    LOWER_VALVE_LEVELS,
    PREPARE_LOWER_PULSE_S,
    PREPARE_LOWER_SETTLE_S,
    CalibrationBundle,
    CalibrationError,
    CalibrationStore,
    LiftCalibrationSession,
    LiftTrial,
    LowerCalibrationSession,
    LowerTrial,
    PrepareLowerSession,
    PrepareLowerState,
    analyze_lift_trials,
    analyze_lower_trials,
)


def sample(timestamp: float, height_mm: float, *, valid: bool = True) -> HeightSample:
    return HeightSample(timestamp, 100, height_mm, valid, None if valid else "bad")


def feedback(timestamp: float, current: int = 100, *, fault: int = 0) -> PumpFeedback:
    return PumpFeedback(timestamp, current, fault, 0)


def complete_lift_trials() -> tuple[LiftTrial, ...]:
    return tuple(
        LiftTrial(
            pwm=40,
            repeat=repeat,
            start_delay_s=0.10 + repeat * 0.02,
            displacement_mm=4.0 + repeat,
            speed_mm_s=(4.0 + repeat) / 0.1,
            coast_mm=1.0 + repeat / 2,
            peak_current_raw=900 + repeat * 10,
            direction_consistent=True,
            success=True,
        )
        for repeat in range(1, 4)
    )


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


def prepare_session(
    *, target: float = 100.0, upper: float = 200.0
) -> PrepareLowerSession:
    return PrepareLowerSession(
        analyze_lift_trials(complete_lift_trials()),
        target_mm=target,
        effective_max_height_mm=upper,
        direction_tolerance_mm=0.5,
        sensor_timeout_s=0.1,
        feedback_timeout_s=0.15,
        current_multiplier=1.5,
        current_duration_s=0.2,
    )


def test_lift_analysis_accepts_delayed_settled_displacement() -> None:
    result = analyze_lift_trials(complete_lift_trials())

    assert LIFT_PWM_LEVELS == tuple(range(40, 81, 5))
    assert result.min_stable_pwm == 40
    assert result.coarse_pwm == 40
    assert result.response_delay_s == pytest.approx(0.16)
    assert result.max_coast_mm == pytest.approx(2.5)
    assert result.peak_current_by_pwm == {40: 930}

    with pytest.raises(CalibrationError, match="40%.*3"):
        analyze_lift_trials(complete_lift_trials()[:-1])


def test_lift_analysis_rejects_response_later_than_controller_limit() -> None:
    delayed = list(complete_lift_trials())
    delayed[1] = replace(delayed[1], start_delay_s=0.301, success=False)

    with pytest.raises(CalibrationError, match="响应延迟.*300 ms"):
        analyze_lift_trials(tuple(delayed))


def test_lift_analysis_rejects_noncanonical_pwm_and_failed_trial() -> None:
    wrong_pwm = list(complete_lift_trials())
    wrong_pwm[1] = replace(wrong_pwm[1], pwm=45)
    with pytest.raises(CalibrationError, match="40%.*3"):
        analyze_lift_trials(tuple(wrong_pwm))

    failed = list(complete_lift_trials())
    failed[1] = replace(failed[1], displacement_mm=0.5, success=False)
    with pytest.raises(CalibrationError, match="至少 1 mm"):
        analyze_lift_trials(tuple(failed))


def test_lower_analysis_requires_exact_plan_and_explicit_measured_comfort_value() -> None:
    result = analyze_lower_trials(complete_lower_trials())

    assert LOWER_VALVE_LEVELS == tuple(range(0x10, 0xA1, 0x10))
    assert result.min_start_valve == 0x30
    assert result.comfortable_valve is None
    with pytest.raises(CalibrationError, match="实测"):
        result.confirm_comfortable(0x35)

    confirmed = result.confirm_comfortable(0x50)

    assert confirmed.comfortable_valve == 0x50


def run_lift_cycle(
    session: LiftCalibrationSession,
    *,
    started_at: float,
    start_height: float,
    stop_height: float,
    first_motion_at: float | None,
    highest_height: float,
    final_height: float,
) -> None:
    """用确定性时间戳推进一次100 ms通电和700 ms全零观察。"""
    assert session.step(
        now=started_at,
        sample=sample(started_at, start_height),
        feedback=feedback(started_at, 900),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=started_at + 0.1,
        sample=sample(started_at + 0.1, stop_height),
        feedback=feedback(started_at + 0.1, 950),
        lift_authorized=True,
    ).lift_pwm == 0
    if first_motion_at is not None:
        assert session.step(
            now=started_at + first_motion_at,
            sample=sample(started_at + first_motion_at, start_height + 0.2),
            feedback=feedback(started_at + first_motion_at, 0),
            lift_authorized=False,
        ).lift_pwm == 0
    assert session.step(
        now=started_at + 0.4,
        sample=sample(started_at + 0.4, highest_height),
        feedback=feedback(started_at + 0.4, 0),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=started_at + 0.8,
        sample=sample(started_at + 0.8, final_height),
        feedback=feedback(started_at + 0.8, 0),
        lift_authorized=False,
    ).lift_pwm == 0


def release_lift_authorization(
    session: LiftCalibrationSession, *, now: float, height: float
) -> None:
    assert session.step(
        now=now,
        sample=sample(now, height),
        feedback=feedback(now, 0),
        lift_authorized=False,
    ).lift_pwm == 0


def prime_lift_session(
    session: LiftCalibrationSession, *, height: float = 0.1
) -> float:
    run_lift_cycle(
        session,
        started_at=0.0,
        start_height=height,
        stop_height=height,
        first_motion_at=None,
        highest_height=height,
        final_height=height,
    )
    release_lift_authorization(session, now=0.81, height=height)
    return 0.82


def test_lift_session_precharges_then_records_three_delayed_measurements() -> None:
    session = LiftCalibrationSession()
    now = prime_lift_session(session)

    assert session.trials == ()
    assert session.done is False

    height = 0.1
    for delay, net, coast in (
        (0.14, 4.0, 4.8),
        (0.16, 4.2, 5.0),
        (0.18, 4.4, 5.2),
    ):
        run_lift_cycle(
            session,
            started_at=now,
            start_height=height,
            stop_height=height,
            first_motion_at=delay,
            highest_height=height + coast,
            final_height=height + net,
        )
        height += net
        release_lift_authorization(session, now=now + 0.81, height=height)
        now += 0.82

    assert session.done
    assert [trial.repeat for trial in session.trials] == [1, 2, 3]
    assert [trial.displacement_mm for trial in session.trials] == pytest.approx(
        [4.0, 4.2, 4.4]
    )
    assert [trial.start_delay_s for trial in session.trials] == pytest.approx(
        [0.14, 0.16, 0.18]
    )
    assert [trial.coast_mm for trial in session.trials] == pytest.approx(
        [4.8, 5.0, 5.2]
    )


def test_lift_session_requires_release_before_every_new_cycle() -> None:
    session = LiftCalibrationSession()
    session.step(
        now=0.0,
        sample=sample(0.0, 0.1),
        feedback=feedback(0.0),
        lift_authorized=True,
    )
    session.step(
        now=0.1,
        sample=sample(0.1, 0.1),
        feedback=feedback(0.1),
        lift_authorized=True,
    )
    session.step(
        now=0.8,
        sample=sample(0.8, 0.1),
        feedback=feedback(0.8),
        lift_authorized=True,
    )

    assert session.step(
        now=0.82,
        sample=sample(0.82, 0.1),
        feedback=feedback(0.82),
        lift_authorized=True,
    ).lift_pwm == 0
    release_lift_authorization(session, now=0.84, height=0.1)
    assert session.step(
        now=0.86,
        sample=sample(0.86, 0.1),
        feedback=feedback(0.86),
        lift_authorized=True,
    ).lift_pwm == 40


def test_lift_session_checks_reverse_motion_during_settle() -> None:
    session = LiftCalibrationSession()
    session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )
    session.step(
        now=0.1,
        sample=sample(0.1, 10.0),
        feedback=feedback(0.1),
        lift_authorized=True,
    )

    command = session.step(
        now=0.3,
        sample=sample(0.3, 9.4),
        feedback=feedback(0.3),
        lift_authorized=False,
    )

    assert command == PumpCommand.safe_stop()
    assert session.failed
    assert "方向反向" in (session.fault_reason or "")


def test_lift_session_accepts_signed_current_and_records_peak_magnitude() -> None:
    """现场负极性的泵电流仍是有效反馈，标定峰值应保存电流幅值。"""
    session = LiftCalibrationSession()
    now = prime_lift_session(session, height=10.0)

    assert session.step(
        now=now,
        sample=sample(now, 10.0),
        feedback=feedback(now, -18),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=now + 0.1,
        sample=sample(now + 0.1, 10.0),
        feedback=feedback(now + 0.1, -120),
        lift_authorized=True,
    ).lift_pwm == 0
    assert session.step(
        now=now + 0.2,
        sample=sample(now + 0.2, 10.2),
        feedback=feedback(now + 0.2, -2000),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=now + 0.8,
        sample=sample(now + 0.8, 12.5),
        feedback=feedback(now + 0.8, -2000),
        lift_authorized=False,
    ).lift_pwm == 0

    assert session.failed is False
    assert session.trials[0].peak_current_raw == 120


def test_lift_session_authorization_loss_immediately_stops_and_restarts_trial() -> None:
    session = LiftCalibrationSession()

    assert session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=0.05,
        sample=sample(0.05, 10.2),
        feedback=feedback(0.05),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=0.2,
        sample=sample(0.2, 10.2),
        feedback=feedback(0.2),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.trials == ()


def test_lift_session_authorization_loss_during_settle_keeps_completed_pulse() -> None:
    session = LiftCalibrationSession()
    now = prime_lift_session(session, height=10.0)
    assert session.step(
        now=now,
        sample=sample(now, 10.0),
        feedback=feedback(now),
        lift_authorized=True,
    ).lift_pwm == 40

    assert session.step(
        now=now + 0.1,
        sample=sample(now + 0.1, 10.0),
        feedback=feedback(now + 0.1),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=now + 0.2,
        sample=sample(now + 0.2, 10.2),
        feedback=feedback(now + 0.2),
        lift_authorized=False,
    ).lift_pwm == 0
    assert session.step(
        now=now + 0.8,
        sample=sample(now + 0.8, 15.0),
        feedback=feedback(now + 0.8),
        lift_authorized=False,
    ).lift_pwm == 0

    assert len(session.trials) == 1
    assert session.trials[0].displacement_mm == pytest.approx(5.0)


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


def test_prepare_lower_uses_40_percent_100ms_pulses_and_700ms_settle() -> None:
    assert PREPARE_LOWER_PULSE_S == 0.1
    assert PREPARE_LOWER_SETTLE_S == 0.7
    session = prepare_session()

    assert session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=False,
    ) == PumpCommand.safe_stop()
    assert session.state is PrepareLowerState.WAIT_AUTH

    assert session.step(
        now=0.02,
        sample=sample(0.02, 10.0),
        feedback=feedback(0.02),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=0.119,
        sample=sample(0.119, 10.0),
        feedback=feedback(0.119),
        lift_authorized=True,
    ).lift_pwm == 40
    assert session.step(
        now=0.12,
        sample=sample(0.12, 10.0),
        feedback=feedback(0.12),
        lift_authorized=True,
    ) == PumpCommand.safe_stop()
    assert session.state is PrepareLowerState.SETTLE

    assert session.step(
        now=0.819,
        sample=sample(0.819, 14.0),
        feedback=feedback(0.819),
        lift_authorized=True,
    ) == PumpCommand.safe_stop()
    assert session.step(
        now=0.82,
        sample=sample(0.82, 14.0),
        feedback=feedback(0.82),
        lift_authorized=True,
    ).lift_pwm == 40


def test_prepare_lower_authorization_loss_stops_and_cannot_bypass_settle() -> None:
    session = prepare_session()
    session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )

    stopped = session.step(
        now=0.04,
        sample=sample(0.04, 10.0),
        feedback=feedback(0.04),
        lift_authorized=False,
    )
    retried = session.step(
        now=0.05,
        sample=sample(0.05, 10.1),
        feedback=feedback(0.05),
        lift_authorized=True,
    )

    assert stopped == PumpCommand.safe_stop()
    assert retried == PumpCommand.safe_stop()
    assert session.state is PrepareLowerState.SETTLE


def test_prepare_lower_completes_at_target_and_never_commands_lower_valve() -> None:
    session = prepare_session()
    first = session.step(
        now=0.0,
        sample=sample(0.0, 99.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )

    command = session.step(
        now=0.04,
        sample=sample(0.04, 100.1),
        feedback=feedback(0.04),
        lift_authorized=True,
    )

    assert first.lower_valve == 0
    assert command == PumpCommand.safe_stop()
    assert session.done is True
    assert session.state is PrepareLowerState.DONE
    assert session.final_height_mm == pytest.approx(100.1)


def test_prepare_lower_requires_room_for_measured_coast() -> None:
    with pytest.raises(CalibrationError, match="上滑.*安全空间"):
        prepare_session(target=198.0, upper=200.0)


def test_prepare_lower_rejects_target_not_above_initial_height() -> None:
    session = prepare_session(target=100.0)

    command = session.step(
        now=0.0,
        sample=sample(0.0, 100.0),
        feedback=feedback(0.0),
        lift_authorized=False,
    )

    assert command == PumpCommand.safe_stop()
    assert session.failed
    assert "高于启动高度" in (session.fault_reason or "")


def test_prepare_lower_latches_reverse_motion() -> None:
    session = prepare_session()
    session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )

    command = session.step(
        now=0.04,
        sample=sample(0.04, 9.4),
        feedback=feedback(0.04),
        lift_authorized=True,
    )

    assert command == PumpCommand.safe_stop()
    assert session.failed
    assert "反向" in (session.fault_reason or "")


def test_prepare_lower_overcurrent_must_persist_for_configured_duration() -> None:
    session = prepare_session()
    threshold_current = 1400

    session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0, threshold_current),
        lift_authorized=True,
    )
    assert not session.failed
    session.step(
        now=0.199,
        sample=sample(0.199, 10.1),
        feedback=feedback(0.199, -threshold_current),
        lift_authorized=True,
    )
    assert not session.failed

    command = session.step(
        now=0.2,
        sample=sample(0.2, 10.1),
        feedback=feedback(0.2, threshold_current),
        lift_authorized=True,
    )

    assert command == PumpCommand.safe_stop()
    assert session.failed
    assert "过流" in (session.fault_reason or "")


def test_prepare_lower_faults_at_effective_hard_limit_before_success() -> None:
    session = prepare_session(target=100.0, upper=102.5)
    session.step(
        now=0.0,
        sample=sample(0.0, 10.0),
        feedback=feedback(0.0),
        lift_authorized=True,
    )

    command = session.step(
        now=0.04,
        sample=sample(0.04, 102.5),
        feedback=feedback(0.04),
        lift_authorized=True,
    )

    assert command == PumpCommand.safe_stop()
    assert session.failed
    assert "有效最大高度" in (session.fault_reason or "")


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


def test_calibration_bundle_accepts_single_level_lift_result() -> None:
    lift = analyze_lift_trials(complete_lift_trials())
    lower = analyze_lower_trials(complete_lower_trials()).confirm_comfortable(0x50)

    bundle = CalibrationBundle.from_results(lift, lower)

    assert bundle.min_stable_pwm == bundle.coarse_pwm == 40
    assert dict(bundle.peak_current_by_pwm) == {40: 930}
    assert CalibrationBundle.from_json_object(bundle.to_json_object()) == bundle


def test_calibration_bundle_still_reads_legacy_complete_peak_map() -> None:
    bundle = CalibrationBundle(
        min_stable_pwm=50,
        coarse_pwm=70,
        response_delay_s=0.15,
        max_coast_mm=5.0,
        peak_current_by_pwm={pwm: pwm * 10 for pwm in LIFT_PWM_LEVELS},
        lower_min_start_valve=0x30,
        lower_comfortable_valve=0x50,
    )

    assert CalibrationBundle.from_json_object(bundle.to_json_object()) == bundle


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
