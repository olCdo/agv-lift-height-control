from __future__ import annotations

import sys
from collections import deque
from dataclasses import dataclass
from inspect import signature
from threading import Event, Lock, Thread as WorkerThread, current_thread
from time import monotonic
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest

import agv_lift_height_control as package
import agv_lift_height_control.can_pump as can_module
from agv_lift_height_control import CanConfig, PumpCommand, PumpFeedback


def api(name: str) -> Any:
    assert hasattr(package, name), f"缺少公共 CAN 接口: {name}"
    return getattr(package, name)


def can_config(**changes: object) -> CanConfig:
    values: dict[str, object] = {
        "interface": "can0",
        "bitrate": 500000,
        "command_id": 0x217,
        "feedback_id": 0x197,
        "nmt_id": 0,
        "send_period_s": 0.05,
        "feedback_timeout_s": 0.15,
        "command_timeout_s": 0.15,
        "preflight_s": 0.3,
        "startup_nmt_s": 5.0,
        "shutdown_zero_frames": 3,
    }
    values.update(changes)
    return CanConfig(**values)  # type: ignore[arg-type]


def test_default_can_factories_import_lazy_python_can_submodules(monkeypatch) -> None:
    """总线与消息工厂不能依赖 ``can.__init__`` 预先导出公共对象。"""
    bus_calls = []
    message_calls = []
    expected_bus = object()
    expected_message = object()

    def create_bus(**kwargs):
        bus_calls.append(kwargs)
        return expected_bus

    def create_message(**kwargs):
        message_calls.append(kwargs)
        return expected_message

    can_package = ModuleType("can")
    can_package.__path__ = []  # type: ignore[attr-defined]
    interface_module = ModuleType("can.interface")
    interface_module.Bus = create_bus  # type: ignore[attr-defined]
    message_module = ModuleType("can.message")
    message_module.Message = create_message  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "can", can_package)
    monkeypatch.setitem(sys.modules, "can.interface", interface_module)
    monkeypatch.setitem(sys.modules, "can.message", message_module)

    assert not hasattr(can_package, "Bus")
    assert not hasattr(can_package, "Message")
    assert can_module._create_socketcan_bus("can0") is expected_bus
    assert can_module._create_can_message(arbitration_id=0x197) is expected_message
    assert bus_calls == [{"interface": "socketcan", "channel": "can0"}]
    assert message_calls == [{"check": True, "arbitration_id": 0x197}]


@dataclass
class FakeMessage:
    arbitration_id: int
    data: Any
    is_extended_id: bool = False
    is_remote_frame: bool = False
    is_error_frame: bool = False
    dlc: int | None = None

    def __post_init__(self) -> None:
        if self.dlc is None:
            self.dlc = len(self.data)


def message_factory(**kwargs: Any) -> FakeMessage:
    return FakeMessage(**kwargs)


class FakeBus:
    """完整保留发送帧，并将预检接收与后台接收分成两个独立队列。"""

    def __init__(
        self,
        *,
        preflight: list[object] | None = None,
        runtime: list[object] | None = None,
        fail_send_numbers: set[int] | None = None,
    ) -> None:
        self.preflight = deque(preflight or [None])
        self.runtime = deque(runtime or [])
        self.fail_send_numbers = fail_send_numbers or set()
        self.sent: list[FakeMessage] = []
        self.send_attempts: list[FakeMessage] = []
        self.recv_timeouts: list[float] = []
        self.shutdown_calls = 0
        self.send_timeouts: list[float | None] = []
        self.closed = Event()
        self._lock = Lock()

    def recv(self, timeout: float) -> object | None:
        self.recv_timeouts.append(timeout)
        queue = self.preflight if current_thread().name == "MainThread" else self.runtime
        with self._lock:
            item = queue.popleft() if queue else None
        if isinstance(item, BaseException):
            raise item
        if item is None and current_thread().name != "MainThread":
            self.closed.wait(min(timeout, 0.01))
            return None
        return item

    def send(self, message: FakeMessage, timeout: float | None = None) -> None:
        with self._lock:
            self.send_timeouts.append(timeout)
            self.send_attempts.append(message)
            number = len(self.send_attempts)
            if number in self.fail_send_numbers:
                raise OSError("simulated CAN send failure")
            self.sent.append(message)

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        self.closed.set()


class TimedPreflightBus(FakeBus):
    """用虚拟时钟证明预检确实覆盖整个窗口，并可在窗口末端注入帧。"""

    def __init__(
        self,
        clock: "ManualClock",
        *,
        arrival_s: float | None = None,
        arrival_frame: FakeMessage | None = None,
    ) -> None:
        super().__init__(preflight=[])
        self.clock = clock
        self.arrival_s = arrival_s
        self.arrival_frame = arrival_frame

    def recv(self, timeout: float) -> object | None:
        if current_thread().name != "MainThread":
            return super().recv(timeout)
        assert self.send_attempts == [], "预检窗口结束前不得发送任何 CAN 帧"
        self.recv_timeouts.append(timeout)
        if (
            self.arrival_frame is not None
            and self.arrival_s is not None
            and self.clock() <= self.arrival_s <= self.clock() + timeout
        ):
            self.clock.advance(self.arrival_s - self.clock())
            frame, self.arrival_frame = self.arrival_frame, None
            return frame
        self.clock.advance(timeout)
        return None


class BlockingReceiveErrorBus(FakeBus):
    """让 stop 与接收线程异常稳定交错，用于验证总线只清理一次。"""

    def __init__(self) -> None:
        super().__init__()
        self.receive_entered = Event()
        self.release_receive = Event()

    def recv(self, timeout: float) -> object | None:
        if current_thread().name == "MainThread":
            return super().recv(timeout)
        self.receive_entered.set()
        self.release_receive.wait(1.0)
        raise OSError("simulated CAN receive failure")


class TimeoutBlockingSendBus(FakeBus):
    """模拟驱动发送阻塞；只有调用者传入 timeout 才能在有限时间内退出。"""

    def __init__(self) -> None:
        super().__init__()
        self.block_commands = False
        self.send_entered = Event()
        self.release_send = Event()

    def send(self, message: FakeMessage, timeout: float | None = None) -> None:
        if self.block_commands and message.arbitration_id == 0x217:
            self.send_entered.set()
            self.release_send.wait(1.0 if timeout is None else timeout)
            raise TimeoutError("simulated CAN send timeout")
        super().send(message, timeout=timeout)


class PausingMessageFactory:
    """只暂停下一帧 0x217 的构造，稳定暴露反馈提交与实际发送之间的交错。"""

    def __init__(self) -> None:
        self.armed = False
        self.entered = Event()
        self.release = Event()

    def arm(self) -> None:
        self.armed = True
        self.entered.clear()
        self.release.clear()

    def __call__(self, **kwargs: Any) -> FakeMessage:
        if self.armed and kwargs["arbitration_id"] == 0x217:
            self.armed = False
            self.entered.set()
            self.release.wait(1.0)
        return FakeMessage(**kwargs)


class ManualClock:
    def __init__(self, value: float = 0.0) -> None:
        self.value = value
        self._lock = Lock()

    def __call__(self) -> float:
        with self._lock:
            return self.value

    def advance(self, seconds: float) -> None:
        with self._lock:
            self.value += seconds


class ControlledSleeper:
    """推进虚拟时钟，在指定次数后阻塞发送线程，避免依赖真实 5 秒等待。"""

    def __init__(self, clock: ManualClock, advances_before_block: int = 0) -> None:
        self.clock = clock
        self.advances_before_block = advances_before_block
        self.calls: list[float] = []
        self.blocked = Event()
        self.release = Event()
        self._lock = Lock()

    def __call__(self, seconds: float) -> None:
        with self._lock:
            self.calls.append(seconds)
            call_number = len(self.calls)
        if call_number <= self.advances_before_block:
            self.clock.advance(seconds)
            return
        self.blocked.set()
        self.release.wait(1.0)


class IndefiniteSleeper:
    """阻塞发送线程，直到测试显式放行，用于稳定复现 join 超时。"""

    def __init__(self) -> None:
        self.entered = Event()
        self.release = Event()

    def __call__(self, _seconds: float) -> None:
        self.entered.set()
        self.release.wait()


class SequenceBusFactory:
    def __init__(self, *buses: FakeBus) -> None:
        self.buses = deque(buses)
        self.calls: list[str] = []

    def __call__(self, interface: str) -> FakeBus:
        self.calls.append(interface)
        return self.buses.popleft()


class BlockingFailureCleanupBus(FakeBus):
    """让接收线程在故障清理的零帧发送中停住，以暴露跨代启动竞态。"""

    def __init__(self) -> None:
        super().__init__()
        self.trigger_failure = Event()
        self.cleanup_entered = Event()
        self.release_cleanup = Event()
        self.block_cleanup = False

    def recv(self, timeout: float) -> object | None:
        if current_thread().name == "MainThread":
            return super().recv(timeout)
        self.trigger_failure.wait()
        raise OSError("simulated receive failure during cleanup")

    def send(self, message: FakeMessage, timeout: float | None = None) -> None:
        if (
            self.block_cleanup
            and current_thread().name == "can-pump-receive"
            and message.arbitration_id == 0x217
        ):
            self.cleanup_entered.set()
            self.release_cleanup.wait()
        super().send(message, timeout=timeout)


def make_pump(
    bus: FakeBus,
    *,
    clock: ManualClock | None = None,
    sleeper: ControlledSleeper | None = None,
    config: CanConfig | None = None,
    factory=message_factory,
):
    clock = clock or ManualClock()
    sleeper = sleeper or ControlledSleeper(clock)
    pump_type = api("CanPump")
    pump = pump_type(
        config or can_config(),
        bus_factory=lambda interface: bus,
        message_factory=factory,
        clock=clock,
        sleeper=sleeper,
        link_checker=lambda interface, bitrate: None,
    )
    return pump, clock, sleeper


def wait_until(predicate, timeout: float = 0.5) -> None:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        Event().wait(0.001)
    raise AssertionError("等待异步 CAN 状态更新超时")


def test_encode_command_and_nmt_payloads_follow_protocol_layout() -> None:
    encode_command = api("encode_pump_command")
    encode_nmt = api("encode_nmt_start")

    assert encode_command(PumpCommand(True, 37, 8, 9, 64)) == bytes([1, 37, 8, 9, 64, 0, 0, 0])
    assert encode_command(PumpCommand.safe_stop()) == bytes(8)
    assert encode_nmt() == bytes([0x01, 0x00])


def test_parse_feedback_extracts_little_endian_fields_and_local_timestamp() -> None:
    parse_feedback = api("parse_pump_feedback")
    frame = FakeMessage(0x197, bytes([9, 8, 7, 0x34, 0x12, 6, 5, 4]))

    feedback = parse_feedback(frame, timestamp=12.5)

    assert feedback == PumpFeedback(12.5, 0x1234, 6, 4)


@pytest.mark.parametrize(
    ("current_bytes", "expected_current"),
    [
        (bytes((0xFD, 0xFF)), -3),
        (bytes((0xFA, 0xFF)), -6),
    ],
)
def test_parse_feedback_treats_motor_current_as_signed_16_bit(
    current_bytes: bytes, expected_current: int
) -> None:
    """零电流附近的负偏置必须保留符号，不能显示成 65533 一类大电流。"""
    frame = FakeMessage(
        0x197,
        bytes((0x10, 0, 0, current_bytes[0], current_bytes[1], 0x52, 0xFF, 0)),
    )

    feedback = api("parse_pump_feedback")(frame, timestamp=1.0)

    assert feedback.current_raw == expected_current


@pytest.mark.parametrize(
    "frame",
    [
        FakeMessage(0x198, bytes(8)),
        FakeMessage(0x197, bytes(8), is_extended_id=True),
        FakeMessage(0x197, bytes(8), is_remote_frame=True),
        FakeMessage(0x197, bytes(8), is_error_frame=True),
        FakeMessage(0x197, bytes(7)),
        FakeMessage(0x197, bytes(8), dlc=7),
        FakeMessage(0x197, bytes(8), dlc=9),
        FakeMessage(0x197, [0, 0, 0, 0, 0, 0, 0, 256]),
        FakeMessage(0x197, [0, 0, 0, 0, 0, 0, 0, True]),
    ],
)
def test_parse_feedback_rejects_wrong_frame_shape(frame: FakeMessage) -> None:
    with pytest.raises(ValueError):
        api("parse_pump_feedback")(frame, timestamp=1.0)


def test_parse_feedback_rejects_invalid_timestamp() -> None:
    with pytest.raises(ValueError, match="timestamp"):
        api("parse_pump_feedback")(FakeMessage(0x197, bytes(8)), timestamp=float("nan"))


def test_parse_feedback_id_is_fixed_and_cannot_be_overridden() -> None:
    parse_feedback = api("parse_pump_feedback")
    frame = FakeMessage(0x123, bytes(8))

    assert "expected_id" not in signature(parse_feedback).parameters
    with pytest.raises((TypeError, ValueError)):
        parse_feedback(frame, timestamp=1.0, expected_id=0x123)


UP_LINK = """2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT
    link/can  promiscuity 0
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
      bitrate 500000 sample-point 0.875
"""


def test_inspect_can_link_is_read_only_and_returns_parsed_state() -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []

    def runner(command: list[str], **kwargs: object):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=UP_LINK, stderr="")

    info = api("inspect_can_link")("can0", 500000, runner=runner)

    assert info.interface == "can0"
    assert info.bitrate == 500000
    assert info.is_up is True
    assert calls == [
        (
            ["ip", "-details", "link", "show", "can0"],
            {"capture_output": True, "text": True, "check": False},
        )
    ]


def test_inspect_can_link_accepts_up_flag_with_unknown_operstate() -> None:
    output = UP_LINK.replace("state UP", "state UNKNOWN")

    info = api("inspect_can_link")(
        "can0",
        500000,
        runner=lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stdout=output, stderr=""),
    )

    assert info.is_up is True


@pytest.mark.parametrize(
    ("result", "message"),
    [
        (SimpleNamespace(returncode=1, stdout="", stderr="Device not found"), "can0"),
        (SimpleNamespace(returncode=0, stdout=UP_LINK.replace(",UP,", ","), stderr=""), "UP"),
        (SimpleNamespace(returncode=0, stdout=UP_LINK.replace("ERROR-ACTIVE", "BUS-OFF"), stderr=""), "BUS-OFF"),
        (SimpleNamespace(returncode=0, stdout=UP_LINK.replace("bitrate 500000", "bitrate 250000"), stderr=""), "500000"),
        (SimpleNamespace(returncode=0, stdout=UP_LINK.replace("bitrate 500000", "sample-point"), stderr=""), "bitrate"),
    ],
)
def test_inspect_can_link_reports_actionable_failures(result: object, message: str) -> None:
    with pytest.raises(api("CanLinkError"), match=message):
        api("inspect_can_link")("can0", 500000, runner=lambda *_args, **_kwargs: result)


def safe_selection(**changes: object):
    values: dict[str, object] = {
        "config": can_config(),
        "now": 10.0,
        "started_at": 0.0,
        "desired": PumpCommand(True, 40, 2, 3, 4),
        "desired_updated_at": 9.95,
        "feedback": PumpFeedback(9.95, 100, 0, 20),
        "thread_fault": None,
    }
    values.update(changes)
    return api("select_safe_command")(**values)


def test_safe_policy_allows_only_fresh_healthy_command_after_startup_window() -> None:
    command, reason = safe_selection()

    assert command == PumpCommand(True, 40, 2, 3, 4)
    assert reason is None


@pytest.mark.parametrize(
    ("changes", "reason_fragment"),
    [
        ({"now": float("nan")}, "时钟"),
        ({"now": 4.99}, "启动"),
        ({"desired_updated_at": None}, "命令"),
        ({"desired_updated_at": 9.8}, "命令"),
        ({"feedback": None}, "反馈"),
        ({"feedback": PumpFeedback(9.8, 100, 0, 20)}, "反馈"),
        ({"feedback": PumpFeedback(9.95, 100, 7, 20)}, "故障码"),
        ({"thread_fault": "发送线程异常"}, "发送线程异常"),
    ],
)
def test_safe_policy_fails_closed(changes: dict[str, object], reason_fragment: str) -> None:
    command, reason = safe_selection(**changes)

    assert command == PumpCommand.safe_stop()
    assert reason_fragment in (reason or "")


INVALID_SAFETY_TIMESTAMPS = [
    pytest.param(float("nan"), id="nan"),
    pytest.param(float("inf"), id="positive-infinity"),
    pytest.param(float("-inf"), id="negative-infinity"),
    pytest.param(-0.001, id="negative"),
    pytest.param("10.0", id="wrong-type"),
    pytest.param(True, id="bool"),
]


def assert_invalid_safety_timestamp_fails_closed(
    *,
    changes: dict[str, object],
    reason_fragment: str,
) -> None:
    try:
        command, reason = safe_selection(**changes)
    except Exception as exc:  # pragma: no cover - 失败输出需要显示原始异常
        pytest.fail(f"非法时间戳必须安全停机，不得抛出算术异常: {exc!r}")
    assert command == PumpCommand.safe_stop()
    assert reason_fragment in (reason or "")


@pytest.mark.parametrize("invalid_timestamp", INVALID_SAFETY_TIMESTAMPS)
def test_safe_policy_rejects_invalid_started_at_before_arithmetic(
    invalid_timestamp: object,
) -> None:
    assert_invalid_safety_timestamp_fails_closed(
        changes={"started_at": invalid_timestamp},
        reason_fragment="启动时间",
    )


@pytest.mark.parametrize("invalid_timestamp", INVALID_SAFETY_TIMESTAMPS)
def test_safe_policy_rejects_invalid_desired_updated_at_before_arithmetic(
    invalid_timestamp: object,
) -> None:
    assert_invalid_safety_timestamp_fails_closed(
        changes={"desired_updated_at": invalid_timestamp},
        reason_fragment="命令时间",
    )


@pytest.mark.parametrize("invalid_timestamp", INVALID_SAFETY_TIMESTAMPS)
def test_safe_policy_rejects_invalid_feedback_timestamp_before_arithmetic(
    invalid_timestamp: object,
) -> None:
    assert_invalid_safety_timestamp_fails_closed(
        changes={"feedback": PumpFeedback(invalid_timestamp, 100, 0, 20)},  # type: ignore[arg-type]
        reason_fragment="反馈时间",
    )


def test_start_preflight_rejects_other_standard_command_without_sending() -> None:
    bus = FakeBus(preflight=[FakeMessage(0x217, bytes(8))])
    pump, _, _ = make_pump(bus)

    with pytest.raises(api("CanPumpError"), match="0x217"):
        pump.start()

    assert bus.send_attempts == []
    assert bus.shutdown_calls == 1
    assert pump.is_running is False


def test_start_records_link_check_failure_without_opening_bus() -> None:
    bus_factory_calls = 0

    def bus_factory(_interface: str) -> FakeBus:
        nonlocal bus_factory_calls
        bus_factory_calls += 1
        return FakeBus()

    pump_type = api("CanPump")
    pump = pump_type(
        can_config(),
        bus_factory=bus_factory,
        message_factory=message_factory,
        link_checker=lambda _interface, _bitrate: (_ for _ in ()).throw(
            api("CanLinkError")("can0 is DOWN")
        ),
    )

    with pytest.raises(api("CanLinkError"), match="DOWN"):
        pump.start()

    assert bus_factory_calls == 0
    assert "DOWN" in (pump.fault_reason or "")


def test_preflight_covers_full_window_and_detects_command_at_299ms() -> None:
    clock = ManualClock()
    bus = TimedPreflightBus(
        clock,
        arrival_s=0.299,
        arrival_frame=FakeMessage(0x217, bytes(8)),
    )
    pump, _, _ = make_pump(bus, clock=clock)

    with pytest.raises(api("CanPumpError"), match="0x217"):
        pump.start()

    assert clock() == pytest.approx(0.299)
    assert bus.send_attempts == []
    assert bus.shutdown_calls == 1


def test_preflight_waits_full_window_before_first_send() -> None:
    clock = ManualClock()
    bus = TimedPreflightBus(clock)
    pump, _, sleeper = make_pump(bus, clock=clock)

    pump.start()
    assert sleeper.blocked.wait(0.5)

    assert clock() == pytest.approx(0.3)
    assert len(bus.sent) >= 2
    sleeper.release.set()
    pump.stop()


@pytest.mark.parametrize(
    ("failing_name", "expected_joined"),
    [("can-pump-send", 0), ("can-pump-receive", 1)],
)
def test_thread_start_failure_rolls_back_bus_and_started_thread(
    monkeypatch,
    failing_name: str,
    expected_joined: int,
) -> None:
    instances: list[object] = []

    class FailingThread:
        def __init__(self, *, target, name: str, daemon: bool) -> None:
            self.target = target
            self.name = name
            self.daemon = daemon
            self.joined = False
            instances.append(self)

        def start(self) -> None:
            if self.name == failing_name:
                raise RuntimeError(f"{self.name} refused to start")

        def join(self, timeout: float | None = None) -> None:
            self.joined = True

    monkeypatch.setattr(can_module, "Thread", FailingThread)
    bus = FakeBus()
    pump, _, _ = make_pump(bus)

    with pytest.raises(api("CanPumpError"), match="线程"):
        pump.start()

    assert pump.is_running is False
    assert pump._stop_event.is_set()
    assert pump._bus is None
    assert pump._started_at is None
    assert pump._send_thread is None
    assert pump._receive_thread is None
    assert bus.shutdown_calls == 1
    assert all(bytes(frame.data) == bytes(8) for frame in bus.send_attempts[-3:])
    assert sum(instance.joined for instance in instances) == expected_joined


@pytest.mark.parametrize("failing_construction", [1, 2])
def test_thread_construction_failure_rolls_back_all_started_state(
    monkeypatch,
    failing_construction: int,
) -> None:
    construction_count = 0

    class ConstructionFailThread:
        def __init__(self, *, target, name: str, daemon: bool) -> None:
            nonlocal construction_count
            construction_count += 1
            if construction_count == failing_construction:
                raise RuntimeError(f"thread construction {construction_count} failed")
            self.target = target
            self.name = name
            self.daemon = daemon

        def start(self) -> None:
            raise AssertionError("全部 Thread 构造成功前不得启动线程")

        def join(self, timeout: float | None = None) -> None:
            raise AssertionError("尚未启动的线程不得 join")

    monkeypatch.setattr(can_module, "Thread", ConstructionFailThread)
    bus = FakeBus()
    pump, _, _ = make_pump(bus)

    with pytest.raises(api("CanPumpError"), match="线程"):
        pump.start()

    assert pump.is_running is False
    assert pump._stop_event.is_set()
    assert pump._bus is None
    assert pump._started_at is None
    assert pump._send_thread is None
    assert pump._receive_thread is None
    assert bus.shutdown_calls == 1
    assert all(bytes(frame.data) == bytes(8) for frame in bus.send_attempts[-3:])


def install_non_waiting_join_threads(monkeypatch) -> list[object]:
    """运行真实线程，但让 join 立即返回，避免超时测试真实等待一秒。"""

    instances: list[object] = []

    class NonWaitingJoinThread:
        def __init__(self, *, target, name: str, daemon: bool) -> None:
            self._thread = WorkerThread(target=target, name=name, daemon=daemon)
            instances.append(self)

        def start(self) -> None:
            self._thread.start()

        def join(self, timeout: float | None = None) -> None:
            self._thread.join(timeout=0)

        def is_alive(self) -> bool:
            return self._thread.is_alive()

    monkeypatch.setattr(can_module, "Thread", NonWaitingJoinThread)
    return instances


def test_stop_join_timeout_retains_old_thread_and_rejects_restart_before_bus_open(
    monkeypatch,
) -> None:
    threads = install_non_waiting_join_threads(monkeypatch)
    old_bus = FakeBus()
    unused_new_bus = FakeBus()
    bus_factory = SequenceBusFactory(old_bus, unused_new_bus)
    link_checks: list[tuple[str, int]] = []
    sleeper = IndefiniteSleeper()
    pump = api("CanPump")(
        can_config(),
        bus_factory=bus_factory,
        message_factory=message_factory,
        clock=ManualClock(),
        sleeper=sleeper,
        link_checker=lambda interface, bitrate: link_checks.append((interface, bitrate)),
    )

    pump.start()
    assert sleeper.entered.wait(0.5)
    old_send_thread = pump._send_thread
    pump.stop()

    try:
        assert old_send_thread is not None and old_send_thread.is_alive()
        assert pump._send_thread is old_send_thread
        assert pump._stop_event.is_set()
        assert "未退出" in (pump.fault_reason or "")
        with pytest.raises(api("CanPumpError"), match="线程.*未退出"):
            pump.start()
        assert bus_factory.calls == ["can0"]
        assert link_checks == [("can0", 500000)]
        assert unused_new_bus.send_attempts == []
        assert unused_new_bus.shutdown_calls == 0
        assert pump._stop_event.is_set()
    finally:
        sleeper.release.set()
        pump.stop()
        wait_until(lambda: all(not thread.is_alive() for thread in threads))


def test_restart_succeeds_after_retained_old_generation_exits() -> None:
    old_bus = FakeBus()
    new_bus = FakeBus()
    bus_factory = SequenceBusFactory(old_bus, new_bus)
    sleeper = IndefiniteSleeper()
    pump = api("CanPump")(
        can_config(),
        bus_factory=bus_factory,
        message_factory=message_factory,
        clock=ManualClock(),
        sleeper=sleeper,
        link_checker=lambda _interface, _bitrate: None,
    )

    pump.start()
    assert sleeper.entered.wait(0.5)
    old_send_thread = pump._send_thread
    old_receive_thread = pump._receive_thread
    old_send_count = len(old_bus.send_attempts)
    pump.stop()

    try:
        assert pump._send_thread is old_send_thread
        sleeper.release.set()
        wait_until(
            lambda: (
                old_send_thread is not None
                and old_receive_thread is not None
                and not old_send_thread.is_alive()
                and not old_receive_thread.is_alive()
            )
        )

        pump.start()

        assert pump.is_running is True
        assert pump._send_thread is not old_send_thread
        assert pump._receive_thread is not old_receive_thread
        assert bus_factory.calls == ["can0", "can0"]
        assert len(old_bus.send_attempts) == old_send_count + 3
        assert [(frame.arbitration_id, bytes(frame.data)) for frame in new_bus.sent[:2]] == [
            (0x000, bytes([1, 0])),
            (0x217, bytes(8)),
        ]
    finally:
        sleeper.release.set()
        pump.stop()


def test_thread_liveness_helper_accepts_finished_test_double() -> None:
    pump, _, _ = make_pump(FakeBus())

    class FinishedThreadDouble:
        pass

    assert pump._thread_is_alive(None) is False
    assert pump._thread_is_alive(FinishedThreadDouble()) is False


def test_failure_cleanup_rejects_restart_before_new_bus_can_open() -> None:
    old_bus = BlockingFailureCleanupBus()
    unused_new_bus = FakeBus()
    bus_factory = SequenceBusFactory(old_bus, unused_new_bus)
    link_checks: list[tuple[str, int]] = []
    sleeper = IndefiniteSleeper()
    pump = api("CanPump")(
        can_config(),
        bus_factory=bus_factory,
        message_factory=message_factory,
        clock=ManualClock(),
        sleeper=sleeper,
        link_checker=lambda interface, bitrate: link_checks.append((interface, bitrate)),
    )
    pump.start()
    assert sleeper.entered.wait(0.5)
    old_bus.block_cleanup = True
    old_bus.trigger_failure.set()
    assert old_bus.cleanup_entered.wait(0.5)
    wait_until(lambda: pump.is_running is False)

    restart_result: list[BaseException | None] = []
    restart_finished = Event()

    def attempt_restart() -> None:
        try:
            pump.start()
        except BaseException as exc:
            restart_result.append(exc)
        else:
            restart_result.append(None)
        finally:
            restart_finished.set()

    restart_thread = WorkerThread(target=attempt_restart)
    restart_thread.start()
    rejected_before_cleanup_release = restart_finished.wait(0.2)

    try:
        assert rejected_before_cleanup_release is True
        assert restart_result and isinstance(restart_result[0], api("CanPumpError"))
        assert "线程" in str(restart_result[0])
        assert bus_factory.calls == ["can0"]
        assert link_checks == [("can0", 500000)]
        assert unused_new_bus.send_attempts == []
        assert unused_new_bus.shutdown_calls == 0
    finally:
        old_bus.release_cleanup.set()
        sleeper.release.set()
        restart_thread.join(timeout=1.0)
        pump.stop()


def test_stop_racing_receive_failure_closes_and_zeroes_bus_only_once() -> None:
    bus = BlockingReceiveErrorBus()
    pump, _, sleeper = make_pump(bus)
    pump.start()
    assert sleeper.blocked.wait(0.5)
    assert bus.receive_entered.wait(0.5)
    sleeper.release.set()

    stopper = WorkerThread(target=pump.stop)
    stopper.start()
    wait_until(lambda: pump.is_running is False)
    bus.release_receive.set()
    stopper.join(timeout=1.0)

    assert stopper.is_alive() is False
    assert bus.shutdown_calls == 1
    assert len(bus.send_attempts) == 5
    assert "接收线程" in (pump.fault_reason or "")


def test_blocked_runtime_send_uses_timeout_so_stop_returns_bounded() -> None:
    bus = TimeoutBlockingSendBus()
    clock = ManualClock()
    sleeper = ControlledSleeper(clock)
    pump, _, _ = make_pump(bus, clock=clock, sleeper=sleeper)
    pump.start()
    assert sleeper.blocked.wait(0.5)

    bus.block_commands = True
    clock.advance(0.05)
    sleeper.release.set()
    assert bus.send_entered.wait(0.5)

    stopper = WorkerThread(target=pump.stop)
    stopper.start()
    stopper.join(timeout=0.5)
    returned_bounded = not stopper.is_alive()
    # 当前实现若未传 timeout 会仍卡在 send；释放替身以保证失败测试不遗留线程。
    bus.release_send.set()
    stopper.join(timeout=1.5)

    assert returned_bounded is True
    assert bus.send_timeouts[:2] == pytest.approx([0.05, 0.05])
    assert bus.shutdown_calls == 1


def test_preflight_is_passive_then_start_sends_one_nmt_and_zero_command() -> None:
    bus = FakeBus(preflight=[None])
    pump, _, sleeper = make_pump(bus)

    pump.start()
    assert sleeper.blocked.wait(0.5)

    assert bus.recv_timeouts[0] == pytest.approx(0.3)
    assert [(frame.arbitration_id, bytes(frame.data)) for frame in bus.sent[:2]] == [
        (0x000, bytes([1, 0])),
        (0x217, bytes(8)),
    ]
    sleeper.release.set()
    pump.stop()


def test_preflight_feedback_is_parsed_but_never_causes_preflight_send() -> None:
    feedback = FakeMessage(0x197, bytes([0, 0, 0, 0x34, 0x12, 0, 0, 9]))
    bus = FakeBus(preflight=[feedback, None])
    pump, _, sleeper = make_pump(bus)

    pump.start()
    assert sleeper.blocked.wait(0.5)

    assert bus.recv_timeouts[:2] == pytest.approx([0.3, 0.3])
    assert pump.last_feedback == PumpFeedback(0.0, 0x1234, 0, 9)
    sleeper.release.set()
    pump.stop()


def test_run_cycle_keeps_zero_during_five_second_window_then_allows_fresh_command() -> None:
    feedback = FakeMessage(0x197, bytes([0, 0, 0, 1, 0, 0, 0, 2]))
    bus = FakeBus(runtime=[feedback])
    pump, clock, sleeper = make_pump(bus)
    pump.start()
    assert sleeper.blocked.wait(0.5)
    wait_until(lambda: pump.last_feedback is not None)

    clock.value = 4.99
    pump.update_command(PumpCommand(True, 55, 4, 5, 6))
    pump.run_cycle(4.99)
    assert bytes(bus.sent[-1].data) == bytes(8)
    assert pump.last_sent_command == PumpCommand.safe_stop()
    assert sum(frame.arbitration_id == 0 for frame in bus.sent) == 1

    clock.value = 5.0
    pump.update_command(PumpCommand(True, 55, 4, 5, 6))
    bus.runtime.append(feedback)
    wait_until(lambda: pump.last_feedback is not None and pump.last_feedback.timestamp == 5.0)
    pump.run_cycle(5.0)
    assert bus.sent[-1].arbitration_id == 0x217
    assert bytes(bus.sent[-1].data) == bytes([1, 55, 4, 5, 6, 0, 0, 0])
    assert pump.last_sent_command == PumpCommand(True, 55, 4, 5, 6)
    assert pump.fault_reason is None

    sleeper.release.set()
    pump.stop()


def test_after_fault_feedback_state_commit_no_later_cycle_sends_nonzero() -> None:
    bus = FakeBus()
    clock = ManualClock()
    sleeper = ControlledSleeper(clock)
    factory = PausingMessageFactory()
    pump, _, _ = make_pump(bus, clock=clock, sleeper=sleeper, factory=factory)
    pump.start()
    assert sleeper.blocked.wait(0.5)

    clock.value = 5.0
    pump.update_command(PumpCommand(True, 60, 4, 5, 6))
    healthy = FakeMessage(0x197, bytes([0, 0, 0, 1, 0, 0, 0, 2]))
    bus.runtime.append(healthy)
    wait_until(lambda: pump.last_feedback is not None and pump.last_feedback.fault_code == 0)

    factory.arm()
    cycle = WorkerThread(target=lambda: pump.run_cycle(5.0))
    cycle.start()
    assert factory.entered.wait(0.5)

    fault = FakeMessage(0x197, bytes([0, 0, 0, 1, 0, 7, 0, 2]))
    bus.runtime.append(fault)
    fault_committed_while_cycle_paused = False
    deadline = monotonic() + 0.1
    while monotonic() < deadline:
        feedback = pump.last_feedback
        if feedback is not None and feedback.fault_code == 7:
            fault_committed_while_cycle_paused = True
            break
        Event().wait(0.001)
    # 已持有发送周期锁的周期可以先完成；保证边界是“反馈状态提交之后”，不是物理帧到达瞬间。
    sent_count_at_commit = len(bus.sent) if fault_committed_while_cycle_paused else None

    factory.release.set()
    cycle.join(timeout=0.5)
    assert cycle.is_alive() is False
    wait_until(lambda: pump.last_feedback is not None and pump.last_feedback.fault_code == 7)
    if sent_count_at_commit is None:
        sent_count_at_commit = len(bus.sent)

    pump.run_cycle(5.0)
    command_frames_after_fault = [
        frame for frame in bus.sent[sent_count_at_commit:] if frame.arbitration_id == 0x217
    ]
    assert command_frames_after_fault
    assert all(bytes(frame.data) == bytes(8) for frame in command_frames_after_fault)

    sleeper.release.set()
    pump.stop()


def test_send_thread_uses_fifty_millisecond_deadlines_and_full_frames() -> None:
    bus = FakeBus()
    clock = ManualClock()
    sleeper = ControlledSleeper(clock, advances_before_block=3)
    pump, _, _ = make_pump(bus, clock=clock, sleeper=sleeper)

    pump.start()
    assert sleeper.blocked.wait(0.5)

    assert sleeper.calls[:3] == pytest.approx([0.05, 0.05, 0.05])
    command_frames = [frame for frame in bus.sent if frame.arbitration_id == 0x217]
    assert len(command_frames) >= 4
    assert all(frame.dlc == 8 and len(frame.data) == 8 for frame in command_frames)
    sleeper.release.set()
    pump.stop()


def test_runtime_receiver_updates_feedback_with_local_clock() -> None:
    frame = FakeMessage(0x197, bytes([0, 0, 0, 0x78, 0x56, 3, 0, 9]))
    bus = FakeBus(runtime=[frame])
    pump, clock, sleeper = make_pump(bus)
    clock.value = 2.5

    pump.start()
    assert sleeper.blocked.wait(0.5)
    wait_until(lambda: pump.last_feedback is not None)

    assert pump.last_feedback == PumpFeedback(2.5, 0x5678, 3, 9)
    sleeper.release.set()
    pump.stop()


def test_send_thread_failure_records_fault_attempts_zero_frames_and_closes_bus() -> None:
    bus = FakeBus(fail_send_numbers={3})
    clock = ManualClock()
    sleeper = ControlledSleeper(clock, advances_before_block=1)
    pump, _, _ = make_pump(bus, clock=clock, sleeper=sleeper)

    pump.start()
    wait_until(lambda: pump.fault_reason is not None and "发送线程" in pump.fault_reason)

    assert pump.is_running is False
    assert len(bus.send_attempts) >= 6
    assert all(bytes(frame.data) == bytes(8) for frame in bus.send_attempts[-3:])
    assert bus.shutdown_calls == 1
    sleeper.release.set()
    pump.stop()


def test_stop_is_idempotent_and_sends_configured_zero_frames_before_shutdown() -> None:
    bus = FakeBus()
    pump, _, sleeper = make_pump(bus, config=can_config(shutdown_zero_frames=4))
    pump.start()
    assert sleeper.blocked.wait(0.5)
    sleeper.release.set()

    pump.stop()
    first_attempt_count = len(bus.send_attempts)
    pump.stop()

    assert len(bus.send_attempts) == first_attempt_count
    assert all(
        frame.arbitration_id == 0x217 and bytes(frame.data) == bytes(8)
        for frame in bus.send_attempts[-4:]
    )
    assert bus.shutdown_calls == 1


def test_context_manager_stops_and_zeroes_when_body_raises() -> None:
    bus = FakeBus()
    pump, _, sleeper = make_pump(bus)

    with pytest.raises(RuntimeError, match="body failed"):
        with pump:
            assert sleeper.blocked.wait(0.5)
            sleeper.release.set()
            raise RuntimeError("body failed")

    assert bus.shutdown_calls == 1
    assert all(bytes(frame.data) == bytes(8) for frame in bus.send_attempts[-3:])
