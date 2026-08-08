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


@pytest.mark.parametrize("reason", ["", None, 1])
def test_trigger_rejects_invalid_reason(reason: object) -> None:
    latch = EmergencyStopLatch()

    with pytest.raises((TypeError, ValueError)):
        latch.trigger(reason)  # type: ignore[arg-type]


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


def test_nonzero_command_cannot_prove_post_trigger_zero_send() -> None:
    latch = EmergencyStopLatch()
    latch.trigger("limit switch")

    latch.record_send_success(PumpCommand(interlock=True))

    assert latch.snapshot().zero_sent_after_trigger is False
    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()


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


def test_transport_updates_are_ignored_while_inactive() -> None:
    latch = EmergencyStopLatch()

    latch.record_transport_fault("stale fault")
    latch.record_transport_recovered()

    assert latch.snapshot().transport_fault is None
