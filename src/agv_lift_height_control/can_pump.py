"""CAN 泵协议、安全门控和线程生命周期。

本模块只读取并验证 Linux CAN 链路状态，不会修改接口、bitrate 或系统网络配置。
所有非零输出都必须同时通过启动窗口、命令新鲜度、反馈新鲜度和故障码门控。
"""

from __future__ import annotations

import re
import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from math import isfinite
from threading import Event, Lock, RLock, Thread, current_thread
from time import monotonic, sleep
from typing import Any

from .config import CanConfig
from .types import PumpCommand, PumpFeedback


class CanLinkError(RuntimeError):
    """CAN 接口不存在、未启用或 bitrate 不符合协议时抛出。"""


class CanPumpError(RuntimeError):
    """CAN 泵无法安全启动或未处于可运行生命周期时抛出。"""


@dataclass(frozen=True)
class CanLinkInfo:
    interface: str
    is_up: bool
    bitrate: int


def inspect_can_link(
    interface: str,
    expected_bitrate: int = 500000,
    *,
    runner: Callable[..., Any] = subprocess.run,
) -> CanLinkInfo:
    """只读检查 SocketCAN 接口；绝不执行 ``ip link set``。

    返回值来自 ``ip -details link show``。失败信息包含接口和期望值，便于运维人员
    在系统侧修复网络配置后重试，而不是由控制进程擅自修改系统状态。
    """
    command = ["ip", "-details", "link", "show", interface]
    try:
        result = runner(command, capture_output=True, text=True, check=False)
    except OSError as exc:
        raise CanLinkError(
            f"无法只读检查 CAN 接口 {interface}: {exc}；请确认已安装 iproute2"
        ) from exc
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "未知错误").strip()
        raise CanLinkError(
            f"CAN 接口 {interface} 不存在或无法读取: {detail}；"
            f"请手工运行 ip -details link show {interface} 检查"
        )

    output = result.stdout or ""
    header = re.search(
        rf"^\d+:\s+{re.escape(interface)}(?:@[^:]+)?:\s*<(?P<flags>[^>]*)>.*?\bstate\s+(?P<state>\S+)",
        output,
        flags=re.MULTILINE,
    )
    if header is None:
        raise CanLinkError(f"无法从 ip 输出识别 CAN 接口 {interface} 状态")
    flags = {flag.strip() for flag in header.group("flags").split(",")}
    # CAN 接口可能没有载波语义，Linux operstate 因而可显示 UNKNOWN；管理 UP 标志
    # 才是接口是否启用的判断依据，控制器 BUS-OFF 等状态在下方独立检查。
    is_up = "UP" in flags
    if not is_up:
        raise CanLinkError(
            f"CAN 接口 {interface} 当前不是 UP；请由系统管理员启用接口后重试"
        )

    controller_state_match = re.search(r"\bcan state\s+([A-Z-]+)\b", output)
    if controller_state_match is None:
        raise CanLinkError(
            f"CAN 接口 {interface} 的 ip 详情缺少 can state；请确认它是 CAN 接口"
        )
    controller_state = controller_state_match.group(1)
    if controller_state not in {"ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE"}:
        raise CanLinkError(
            f"CAN 接口 {interface} 控制器状态为 {controller_state}，当前不可安全发送"
        )

    bitrate_match = re.search(r"\bbitrate\s+(\d+)\b", output)
    if bitrate_match is None:
        raise CanLinkError(
            f"CAN 接口 {interface} 的 ip 详情缺少 bitrate；请确认它是 CAN 接口"
        )
    bitrate = int(bitrate_match.group(1))
    if bitrate != expected_bitrate:
        raise CanLinkError(
            f"CAN 接口 {interface} bitrate={bitrate}，协议要求 {expected_bitrate}；"
            "请在系统网络配置中修正后重试"
        )
    return CanLinkInfo(interface=interface, is_up=True, bitrate=bitrate)


def encode_pump_command(command: PumpCommand) -> bytes:
    """编码标准帧 0x217 的固定 DLC8 负载。"""
    if not isinstance(command, PumpCommand):
        raise TypeError("command 必须是 PumpCommand")
    return bytes(
        (
            1 if command.interlock else 0,
            command.lift_pwm,
            command.accel,
            command.decel,
            command.lower_valve,
            0,
            0,
            0,
        )
    )


def encode_nmt_start() -> bytes:
    """编码 CANopen NMT Start Remote Node（广播节点 0）。"""
    return bytes((0x01, 0x00))


def parse_pump_feedback(
    frame: Any,
    *,
    timestamp: float,
) -> PumpFeedback:
    """解析标准数据帧 0x197，并使用本机单调时钟标记接收时刻。

    泵电机电流位于 Byte3..4，采用小端有符号 16 位；零输出时出现的轻微
    负偏置必须保留符号，避免 ``0xFFFD`` 被误报为 65533。
    """
    if type(timestamp) not in {int, float} or not isfinite(float(timestamp)) or timestamp < 0:
        raise ValueError("timestamp 必须是有限且非负的本机单调时钟值")
    if type(getattr(frame, "arbitration_id", None)) is not int:
        raise ValueError("反馈帧 arbitration_id 无效")
    if frame.arbitration_id != 0x197:
        raise ValueError("反馈帧 ID 必须是固定协议值 0x197")
    if getattr(frame, "is_extended_id", None) is not False:
        raise ValueError("反馈必须是标准帧，不能是扩展帧")
    if getattr(frame, "is_remote_frame", None) is not False:
        raise ValueError("反馈不能是远程帧")
    if getattr(frame, "is_error_frame", None) is not False:
        raise ValueError("反馈不能是错误帧")
    if type(getattr(frame, "dlc", None)) is not int or frame.dlc != 8:
        raise ValueError("反馈帧 DLC 必须等于 8")
    data = getattr(frame, "data", None)
    if not isinstance(data, (bytes, bytearray, list, tuple)) or len(data) != 8:
        raise ValueError("反馈帧必须包含 8 个字节")
    if any(type(byte) is not int or not 0 <= byte <= 0xFF for byte in data):
        raise ValueError("反馈帧包含异常字节")
    return PumpFeedback(
        timestamp=float(timestamp),
        current_raw=int.from_bytes(bytes(data[3:5]), "little", signed=True),
        fault_code=data[5],
        lower_current_raw=data[7],
    )


def select_safe_command(
    *,
    config: CanConfig,
    now: float,
    started_at: float,
    desired: PumpCommand,
    desired_updated_at: float | None,
    feedback: PumpFeedback | None,
    thread_fault: str | None,
) -> tuple[PumpCommand, str | None]:
    """纯策略函数：任一安全条件不满足就返回完整全零命令和可查询原因。"""
    safe_stop = PumpCommand.safe_stop()
    if not _is_valid_safety_timestamp(now):
        return safe_stop, "本机时钟值无效，强制全零"
    if not _is_valid_safety_timestamp(started_at):
        return safe_stop, "CAN 泵启动时间戳无效，强制全零"
    if desired_updated_at is not None and not _is_valid_safety_timestamp(desired_updated_at):
        return safe_stop, "泵命令时间戳无效，强制全零"
    if feedback is not None and not _is_valid_safety_timestamp(feedback.timestamp):
        return safe_stop, "CAN 泵反馈时间戳无效，强制全零"
    if thread_fault is not None:
        return safe_stop, thread_fault
    if now < started_at or now - started_at < config.startup_nmt_s:
        return safe_stop, "CAN 泵处于启动 NMT 安全窗口，强制全零"
    if desired_updated_at is None:
        return safe_stop, "尚未收到泵命令，强制全零"
    command_age = now - desired_updated_at
    if command_age < 0:
        return safe_stop, "本机时钟回退导致泵命令时间异常，强制全零"
    if command_age > config.command_timeout_s:
        return safe_stop, "泵命令已过期，强制全零"
    if feedback is None:
        return safe_stop, "尚未收到 CAN 泵反馈，强制全零"
    feedback_age = now - feedback.timestamp
    if feedback_age < 0:
        return safe_stop, "本机时钟回退导致 CAN 反馈时间异常，强制全零"
    if feedback_age > config.feedback_timeout_s:
        return safe_stop, "CAN 泵反馈已超时，强制全零"
    if feedback.fault_code != 0:
        return safe_stop, f"CAN 泵反馈故障码 {feedback.fault_code}，强制全零"
    return desired, None


def _is_valid_safety_timestamp(value: object) -> bool:
    """安全策略时间戳只能是有限、非负的实数，且布尔值不得冒充整数。"""
    return type(value) in {int, float} and isfinite(float(value)) and value >= 0


class CanPump:
    """线程安全的 CAN 泵发送/接收器。

    调用链为 ``update_command -> run_cycle -> encode -> bus.send``；接收线程只接受
    完整 0x197 标准数据帧。进程退出后无法继续发帧，因此 ``stop`` 只能在总线仍可用时
    同步尽力补发配置数量的零帧，不能替代硬件急停或驱动器自身的通信超时保护。
    """

    def __init__(
        self,
        config: CanConfig,
        *,
        bus_factory: Callable[[str], Any] | None = None,
        message_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = monotonic,
        sleeper: Callable[[float], None] = sleep,
        link_checker: Callable[[str, int], Any] = inspect_can_link,
    ) -> None:
        self._config = config
        self._bus_factory = bus_factory or _create_socketcan_bus
        self._message_factory = message_factory or _create_can_message
        self._clock = clock
        self._sleeper = sleeper
        self._link_checker = link_checker

        self._state_lock = RLock()
        self._lifecycle_lock = Lock()
        self._send_lock = Lock()
        self._cycle_lock = Lock()
        # 锁序约束：同时需要周期状态和共享状态时，始终先 cycle、后 state；
        # 工作线程不获取 lifecycle，避免 stop join 与故障清理互相等待。
        self._stop_event = Event()
        self._threads_ready = Event()
        self._bus: Any | None = None
        self._send_thread: Thread | None = None
        self._receive_thread: Thread | None = None
        self._running = False
        self._started_at: float | None = None
        self._nmt_sent = False
        self._desired = PumpCommand.safe_stop()
        self._desired_updated_at: float | None = None
        self._last_sent_command = PumpCommand.safe_stop()
        self._last_feedback: PumpFeedback | None = None
        self._fault_reason: str | None = "CAN 泵尚未启动"
        self._thread_fault: str | None = None

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return self._running and self._bus is not None and not self._stop_event.is_set()

    @property
    def desired_command(self) -> PumpCommand:
        with self._state_lock:
            return self._desired

    @property
    def last_feedback(self) -> PumpFeedback | None:
        with self._state_lock:
            return self._last_feedback

    @property
    def last_sent_command(self) -> PumpCommand:
        """返回最近一次成功写入 0x217 的门控后命令，供 TUI/CSV 记录实际输出。"""
        with self._state_lock:
            return self._last_sent_command

    @property
    def fault_reason(self) -> str | None:
        with self._state_lock:
            return self._fault_reason

    @property
    def thread_fault(self) -> str | None:
        """向前台主循环暴露发送/接收线程的锁存异常。"""
        with self._state_lock:
            return self._thread_fault

    def update_command(self, command: PumpCommand) -> None:
        """原子保存期望命令和更新时间；启动前调用安全，但命令仍会按超时失效。"""
        if not isinstance(command, PumpCommand):
            raise TypeError("command 必须是 PumpCommand")
        timestamp = self._clock()
        with self._state_lock:
            self._desired = command
            self._desired_updated_at = timestamp

    def start(self) -> None:
        """只读校验链路、被动预检，再发送一次 NMT 并启动双线程。"""
        with self._lifecycle_lock:
            if self.is_running:
                return
            with self._state_lock:
                previous_send_thread = self._send_thread
                previous_receive_thread = self._receive_thread
            live_thread_names = [
                name
                for name, thread in (
                    ("发送线程", previous_send_thread),
                    ("接收线程", previous_receive_thread),
                )
                if self._thread_is_alive(thread)
            ]
            if live_thread_names:
                reason = f"CAN 后台线程仍未退出，拒绝重启: {', '.join(live_thread_names)}"
                self._stop_event.set()
                with self._state_lock:
                    self._running = False
                    self._fault_reason = reason
                raise CanPumpError(reason)
            if previous_send_thread is not None or previous_receive_thread is not None:
                with self._state_lock:
                    self._send_thread = None
                    self._receive_thread = None
            try:
                self._link_checker(self._config.interface, self._config.bitrate)
            except Exception as exc:
                with self._state_lock:
                    self._fault_reason = f"CAN 链路只读检查失败: {exc}"
                raise
            try:
                bus = self._bus_factory(self._config.interface)
            except Exception as exc:
                with self._state_lock:
                    self._fault_reason = f"打开 CAN 接口失败: {exc}"
                raise CanPumpError(self._fault_reason) from exc

            with self._state_lock:
                self._last_feedback = None
            try:
                self._run_preflight(bus)
            except Exception as exc:
                reason = str(exc)
                with self._state_lock:
                    self._fault_reason = reason
                self._close_bus(bus)
                if isinstance(exc, CanPumpError):
                    raise
                raise CanPumpError(f"CAN 预检失败: {reason}") from exc

            started_at = self._clock()
            self._stop_event.clear()
            self._threads_ready.clear()
            with self._state_lock:
                self._bus = bus
                self._running = True
                self._started_at = started_at
                self._nmt_sent = False
                self._thread_fault = None
                self._last_sent_command = PumpCommand.safe_stop()
                self._fault_reason = "CAN 泵处于启动 NMT 安全窗口，强制全零"

            # 预检结束后才允许第一次发送；启动即发 NMT，0x217 同周期保持全零。
            try:
                self.run_cycle(started_at)
            except Exception as exc:
                self._handle_thread_failure("启动发送", exc)
                raise CanPumpError(f"CAN 泵启动发送失败: {exc}") from exc

            started_threads: list[Thread] = []
            try:
                # 两个对象的构造和两次 start 属于同一个启动事务；任何一步失败都走
                # 同一条归零、关闭和状态清理路径。
                self._send_thread = Thread(target=self._send_loop, name="can-pump-send", daemon=True)
                self._receive_thread = Thread(
                    target=self._receive_loop,
                    name="can-pump-receive",
                    daemon=True,
                )
                self._send_thread.start()
                started_threads.append(self._send_thread)
                self._receive_thread.start()
                started_threads.append(self._receive_thread)
            except Exception as exc:
                reason = f"CAN 后台线程启动失败: {exc}"
                self._stop_event.set()
                self._threads_ready.set()
                with self._state_lock:
                    self._running = False
                    self._thread_fault = reason
                    self._fault_reason = reason
                for thread in started_threads:
                    thread.join(timeout=1.0)
                send_thread_alive = self._thread_is_alive(self._send_thread)
                receive_thread_alive = self._thread_is_alive(self._receive_thread)
                with self._cycle_lock:
                    self._send_shutdown_zero_frames(bus)
                    self._close_bus(bus)
                    with self._state_lock:
                        if self._bus is bus:
                            self._bus = None
                        self._started_at = None
                        if not send_thread_alive:
                            self._send_thread = None
                        if not receive_thread_alive:
                            self._receive_thread = None
                        if send_thread_alive or receive_thread_alive:
                            self._fault_reason = f"{reason}；已有线程未退出"
                raise CanPumpError(reason) from exc
            self._threads_ready.set()

    def run_cycle(self, now: float | None = None) -> PumpCommand:
        """执行一个确定性发送周期，并返回实际发出的门控后命令。"""
        timestamp = self._clock() if now is None else now
        if type(timestamp) not in {int, float} or not isfinite(float(timestamp)):
            raise ValueError("now 必须是有限数字")
        timestamp = float(timestamp)
        with self._cycle_lock:
            with self._state_lock:
                bus = self._bus
                started_at = self._started_at
                if not self._running or bus is None or started_at is None:
                    raise CanPumpError("CAN 泵尚未启动")
                desired = self._desired
                desired_updated_at = self._desired_updated_at
                feedback = self._last_feedback
                thread_fault = self._thread_fault
                nmt_sent = self._nmt_sent

            command, reason = select_safe_command(
                config=self._config,
                now=timestamp,
                started_at=started_at,
                desired=desired,
                desired_updated_at=desired_updated_at,
                feedback=feedback,
                thread_fault=thread_fault,
            )
            with self._state_lock:
                self._fault_reason = reason

            if not nmt_sent:
                self._send_payload(bus, self._config.nmt_id, encode_nmt_start())
                with self._state_lock:
                    self._nmt_sent = True
            self._send_payload(bus, self._config.command_id, encode_pump_command(command))
            with self._state_lock:
                self._last_sent_command = command
            return command

    def stop(self) -> None:
        """幂等停机：先停止线程，再尽力同步发送零帧，最后关闭总线。"""
        with self._lifecycle_lock:
            with self._state_lock:
                send_thread = self._send_thread
                receive_thread = self._receive_thread
                if self._bus is None and send_thread is None and receive_thread is None:
                    self._running = False
                    return
                self._running = False
            self._stop_event.set()
            self._threads_ready.set()

            for thread in (send_thread, receive_thread):
                if thread is not None and thread is not current_thread():
                    thread.join(timeout=1.0)

            send_thread_alive = self._thread_is_alive(send_thread)
            receive_thread_alive = self._thread_is_alive(receive_thread)
            live_thread_names = [
                name
                for name, is_alive in (
                    ("发送线程", send_thread_alive),
                    ("接收线程", receive_thread_alive),
                )
                if is_alive
            ]

            # 线程故障可能在 join 期间已关闭总线，因此必须重新读取所有权，不能使用
            # stop 入口处的旧快照重复发送和 shutdown。
            with self._cycle_lock:
                with self._state_lock:
                    bus = self._bus
                if bus is not None:
                    self._send_shutdown_zero_frames(bus)
                    self._close_bus(bus)
                with self._state_lock:
                    if self._bus is bus:
                        self._bus = None
                    self._started_at = None
                    self._send_thread = send_thread if send_thread_alive else None
                    self._receive_thread = receive_thread if receive_thread_alive else None
                    if live_thread_names:
                        self._fault_reason = (
                            "CAN 泵已停止，但后台线程未退出: " + ", ".join(live_thread_names)
                        )
                    elif self._thread_fault is None:
                        self._fault_reason = "CAN 泵已停止并切换为全零"

    def __enter__(self) -> "CanPump":
        self.start()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.stop()

    def _run_preflight(self, bus: Any) -> None:
        """打开总线后只接收不发送；发现任意标准 0x217 即拒绝竞争控制。"""
        deadline = self._clock() + self._config.preflight_s
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0:
                return
            frame = bus.recv(timeout=remaining)
            if frame is None:
                return
            arbitration_id = getattr(frame, "arbitration_id", None)
            if arbitration_id == self._config.command_id and getattr(frame, "is_extended_id", None) is False:
                raise CanPumpError(
                    f"预检期间发现其他进程发送标准帧 0x{self._config.command_id:03X}，拒绝启动"
                )
            if arbitration_id == self._config.feedback_id:
                feedback = parse_pump_feedback(
                    frame,
                    timestamp=self._clock(),
                )
                with self._state_lock:
                    self._last_feedback = feedback

    def _send_loop(self) -> None:
        """按绝对截止时间调度，避免每周期执行耗时持续累积漂移。"""
        self._threads_ready.wait()
        if self._stop_event.is_set():
            return
        next_deadline = self._clock() + self._config.send_period_s
        try:
            while not self._stop_event.is_set():
                now = self._clock()
                delay = next_deadline - now
                if delay > 0:
                    self._sleeper(delay)
                    if self._stop_event.is_set():
                        break
                    now = self._clock()
                    if now < next_deadline:
                        continue
                self.run_cycle(now)
                next_deadline += self._config.send_period_s
                if next_deadline <= now:
                    missed = int((now - next_deadline) / self._config.send_period_s) + 1
                    next_deadline += missed * self._config.send_period_s
        except Exception as exc:
            self._handle_thread_failure("发送线程", exc)

    def _receive_loop(self) -> None:
        """持续接收 0x197；目标帧结构异常也按线程故障立即归零。"""
        self._threads_ready.wait()
        if self._stop_event.is_set():
            return
        try:
            while not self._stop_event.is_set():
                with self._state_lock:
                    bus = self._bus
                if bus is None:
                    return
                frame = bus.recv(timeout=self._config.send_period_s)
                if frame is None:
                    continue
                if getattr(frame, "arbitration_id", None) != self._config.feedback_id:
                    continue
                feedback = parse_pump_feedback(
                    frame,
                    timestamp=self._clock(),
                )
                self._commit_feedback(feedback)
        except Exception as exc:
            self._handle_thread_failure("接收线程", exc)

    def _commit_feedback(self, feedback: PumpFeedback) -> None:
        """提交运行期反馈；与发送周期串行，禁止故障提交后再发送旧健康快照。"""
        with self._cycle_lock:
            with self._state_lock:
                self._last_feedback = feedback

    def _handle_thread_failure(self, source: str, exc: Exception) -> None:
        """仅由首个线程故障执行归零和关闭，避免双线程重复处置同一总线。"""
        # 与 run_cycle 串行：故障一旦写入状态，就不会再有旧快照覆盖原因或发送非零帧。
        with self._cycle_lock:
            with self._state_lock:
                if self._thread_fault is not None:
                    return
                reason = f"CAN {source}异常: {exc}"
                self._thread_fault = reason
                self._fault_reason = reason
                self._running = False
                bus = self._bus
            self._stop_event.set()
            self._threads_ready.set()
            if bus is not None:
                self._send_shutdown_zero_frames(bus)
                self._close_bus(bus)
                with self._state_lock:
                    if self._bus is bus:
                        self._bus = None

    def _send_shutdown_zero_frames(self, bus: Any) -> None:
        """逐帧尽力发送；一次失败不阻止后续零帧尝试。"""
        safe_stop = PumpCommand.safe_stop()
        payload = encode_pump_command(safe_stop)
        sent = False
        for _ in range(self._config.shutdown_zero_frames):
            try:
                self._send_payload(bus, self._config.command_id, payload)
                sent = True
            except Exception:
                pass
        if sent:
            with self._state_lock:
                self._last_sent_command = safe_stop

    @staticmethod
    def _thread_is_alive(thread: object | None) -> bool:
        """真实线程使用 is_alive；无该接口的轻量测试替身按已结束处理。"""
        if thread is None:
            return False
        is_alive = getattr(thread, "is_alive", None)
        if not callable(is_alive):
            return False
        try:
            return bool(is_alive())
        except Exception:
            # 无法确认线程已结束时必须按仍存活处理，避免清停止事件导致跨代复活。
            return True

    def _send_payload(self, bus: Any, arbitration_id: int, data: bytes) -> None:
        message = self._message_factory(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )
        with self._send_lock:
            # python-can 的 send(timeout=...) 在发送队列拥塞时有界失败，避免线程持锁
            # 无限阻塞，导致 stop 无法补发零帧或关闭总线。
            bus.send(message, timeout=self._config.send_period_s)

    @staticmethod
    def _close_bus(bus: Any) -> None:
        """优先使用 python-can 的 shutdown，替身或其他实现可退化到 close。"""
        try:
            shutdown = getattr(bus, "shutdown", None)
            if callable(shutdown):
                shutdown()
                return
            close = getattr(bus, "close", None)
            if callable(close):
                close()
        except Exception:
            pass


def _create_socketcan_bus(interface: str) -> Any:
    """打开既有 SocketCAN 接口；bitrate 已由只读预检确认，不在这里配置。"""
    # 某些 python-can 4.x 包采用延迟导出，顶层 ``can.Bus`` 不一定存在。
    from can.interface import Bus

    return Bus(interface="socketcan", channel=interface)


def _create_can_message(**kwargs: Any) -> Any:
    # 与总线工厂相同，直接导入定义模块，避免依赖顶层包的导出时机。
    from can.message import Message

    return Message(check=True, **kwargs)
