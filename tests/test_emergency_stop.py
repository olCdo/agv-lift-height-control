from dataclasses import FrozenInstanceError

import pytest

from agv_lift_height_control import (
    EmergencyStopLatch,
    EmergencyStopSnapshot,
    PumpCommand,
)


def test_initial_snapshot_is_inactive_and_immutable() -> None:
    snapshot = EmergencyStopLatch().snapshot()

    assert snapshot == EmergencyStopSnapshot(
        active=False,
        reason=None,
        triggered_at=None,
        zero_sent_after_trigger=False,
        transport_fault=None,
    )
    with pytest.raises(FrozenInstanceError):
        snapshot.active = True  # type: ignore[misc]


def test_trigger_latches_first_reason_and_time() -> None:
    clock_calls: list[float] = []

    def clock() -> float:
        value = 12.5 + len(clock_calls)
        clock_calls.append(value)
        return value

    latch = EmergencyStopLatch(clock=clock)

    latch.trigger("operator emergency stop")
    latch.trigger("later controller fault")

    snapshot = latch.snapshot()
    assert snapshot.active is True
    assert snapshot.reason == "operator emergency stop"
    assert snapshot.triggered_at == 12.5
    assert clock_calls == [12.5]


@pytest.mark.parametrize("reason", ["", " ", "\t\n", None, 1])
def test_trigger_rejects_invalid_reason(reason: object) -> None:
    latch = EmergencyStopLatch()

    with pytest.raises((TypeError, ValueError)):
        latch.trigger(reason)  # type: ignore[arg-type]


def test_clock_exception_does_not_partially_trigger_and_retry_can_succeed() -> None:
    calls = 0

    def clock() -> float:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise RuntimeError("clock unavailable")
        return 8.5

    latch = EmergencyStopLatch(clock=clock)

    with pytest.raises(RuntimeError, match="clock unavailable"):
        latch.trigger("first attempt")

    assert latch.snapshot() == EmergencyStopSnapshot(False, None, None, False, None)

    latch.trigger("second attempt")

    assert latch.snapshot() == EmergencyStopSnapshot(True, "second attempt", 8.5, False, None)


@pytest.mark.parametrize(
    ("invalid_time", "error_type"),
    [
        (True, TypeError),
        ("1.0", TypeError),
        (None, TypeError),
        (float("nan"), ValueError),
        (float("inf"), ValueError),
        (float("-inf"), ValueError),
        (-0.1, ValueError),
    ],
)
def test_invalid_clock_result_does_not_partially_trigger_and_retry_can_succeed(
    invalid_time: object,
    error_type: type[Exception],
) -> None:
    clock_results = iter([invalid_time, 9.5])
    latch = EmergencyStopLatch(clock=lambda: next(clock_results))  # type: ignore[arg-type]

    with pytest.raises(error_type):
        latch.trigger("first attempt")

    assert latch.snapshot() == EmergencyStopSnapshot(False, None, None, False, None)

    latch.trigger("second attempt")

    assert latch.snapshot() == EmergencyStopSnapshot(True, "second attempt", 9.5, False, None)


def test_clear_is_idempotent_while_inactive() -> None:
    latch = EmergencyStopLatch()

    latch.clear()
    latch.clear()

    assert latch.snapshot().active is False


def test_clear_rejects_active_latch_without_post_trigger_zero_send() -> None:
    latch = EmergencyStopLatch()
    latch.record_send_success(PumpCommand.safe_stop())
    latch.trigger("limit switch")

    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()

    assert latch.snapshot().active is True


@pytest.mark.parametrize(
    "command",
    [
        PumpCommand(interlock=True),
        PumpCommand(lift_pwm=1),
        PumpCommand(accel=1),
        PumpCommand(decel=1),
        PumpCommand(lower_valve=1),
    ],
)
def test_nonzero_command_cannot_prove_post_trigger_zero_send(
    command: PumpCommand,
) -> None:
    latch = EmergencyStopLatch()
    latch.trigger("limit switch")

    latch.record_send_success(command)

    assert latch.snapshot().zero_sent_after_trigger is False
    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()


def test_nonzero_send_after_zero_send_revokes_clear_evidence() -> None:
    latch = EmergencyStopLatch()
    latch.trigger("limit switch")
    latch.record_send_success(PumpCommand.safe_stop())

    latch.record_send_success(PumpCommand(lift_pwm=1))

    assert latch.snapshot().zero_sent_after_trigger is False
    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()


def test_send_gate_returns_zero_and_trigger_snapshot_while_latched() -> None:
    latch = EmergencyStopLatch()
    latch.trigger("安全回路断开")

    with latch.gate_command_for_send(PumpCommand(lift_pwm=80)) as gate:
        assert gate.command == PumpCommand.safe_stop()
        assert gate.emergency_stop.active is True
        assert gate.emergency_stop.reason == "安全回路断开"


def test_transport_fault_must_be_explicitly_recovered_before_clear() -> None:
    latch = EmergencyStopLatch()
    latch.trigger("sensor timeout")
    latch.record_send_success(PumpCommand.safe_stop())
    latch.record_transport_fault("CAN send failed")

    with pytest.raises(RuntimeError, match="传输正常"):
        latch.clear()

    faulted = latch.snapshot()
    assert faulted.transport_fault == "CAN send failed"
    assert faulted.zero_sent_after_trigger is True

    latch.record_transport_recovered()
    latch.clear()

    assert latch.snapshot() == EmergencyStopSnapshot(False, None, None, False, None)


@pytest.mark.parametrize("reason", ["", " ", "\t\n", None, 1])
def test_active_transport_fault_rejects_invalid_reason_without_losing_fault(
    reason: object,
) -> None:
    latch = EmergencyStopLatch()
    latch.trigger("sensor timeout")
    latch.record_transport_fault("CAN send failed")

    with pytest.raises((TypeError, ValueError)):
        latch.record_transport_fault(reason)  # type: ignore[arg-type]

    assert latch.snapshot().transport_fault == "CAN send failed"


def test_transport_updates_are_ignored_while_inactive() -> None:
    latch = EmergencyStopLatch()

    latch.record_transport_fault("stale fault")
    assert latch.snapshot().transport_fault is None

    latch.record_transport_recovered()

    assert latch.snapshot().transport_fault is None
