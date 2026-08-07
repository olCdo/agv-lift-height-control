"""SSH 前台运行所需的终端、授权、采样线程、日志和停机原语。"""

from __future__ import annotations

import csv
import os
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from itertools import count
from math import ceil
from pathlib import Path
from threading import Event, Lock, Thread
from time import monotonic, sleep, time
from typing import Any, TextIO
from uuid import uuid4

from .types import HeightSample, HeightSource, PumpCommand, PumpFeedback


def validate_foreground_terminal(
    *,
    stdin: Any = sys.stdin,
    stdout: Any = sys.stdout,
    environ: Mapping[str, str] | None = None,
    foreground_checker: Callable[[Any], bool] | None = None,
) -> None:
    """只允许真实前台 TTY，拒绝 tmux、screen、nohup 与 dumb 终端。"""
    environment = os.environ if environ is None else environ
    if not callable(getattr(stdin, "isatty", None)) or not stdin.isatty():
        raise RuntimeError("stdin 不是前台 TTY；不支持 nohup、管道或后台运行")
    if not callable(getattr(stdout, "isatty", None)) or not stdout.isatty():
        raise RuntimeError("stdout 不是前台 TTY；不支持 nohup、重定向或后台运行")
    checker = foreground_checker or _is_foreground_process_group
    if not checker(stdin):
        raise RuntimeError("stdin 属于后台进程组；不支持后台运行")
    if "TMUX" in environment:
        raise RuntimeError("不支持 tmux；请直接使用 SSH 前台终端")
    if "STY" in environment:
        raise RuntimeError("不支持 screen；请直接使用 SSH 前台终端")
    if environment.get("TERM", "").strip().lower() == "dumb":
        raise RuntimeError("不支持 TERM=dumb")


def _is_foreground_process_group(stdin: Any) -> bool:
    """POSIX 上确认当前进程组拥有终端；非 POSIX 测试环境保持可导入。"""
    if not all(hasattr(os, name) for name in ("tcgetpgrp", "getpgrp")):
        return True
    fileno = getattr(stdin, "fileno", None)
    if not callable(fileno):
        return True
    try:
        return os.tcgetpgrp(fileno()) == os.getpgrp()
    except OSError:
        return False


_terminal_event_sequence = count()


@dataclass(frozen=True)
class TerminalEvent:
    """一个独立字符输入事件；序号防止自动重复被合并成持续授权。"""

    kind: str
    key: str | None = None
    sequence: int = field(default_factory=lambda: next(_terminal_event_sequence))

    @classmethod
    def keypress(cls, key: str) -> "TerminalEvent":
        if type(key) is not str or len(key) != 1:
            raise ValueError("终端按键必须是单个字符")
        return cls("key", key)


EOF_EVENT = TerminalEvent("eof")


class PosixAnsiTerminal:
    """延迟导入 POSIX 终端模块的最小 ANSI TUI。"""

    def __init__(self, *, stdin: Any = sys.stdin, stdout: Any = sys.stdout) -> None:
        self.stdin = stdin
        self.stdout = stdout
        self._opened = False
        self._original_attributes: Any = None

    def open(self) -> None:
        validate_foreground_terminal(stdin=self.stdin, stdout=self.stdout)
        if self._opened:
            return
        # Windows 没有 termios/tty，因此只能在真正启用 POSIX TUI 时导入。
        import termios
        import tty

        descriptor = self.stdin.fileno()
        self._original_attributes = termios.tcgetattr(descriptor)
        tty.setcbreak(descriptor)
        self._opened = True
        self.stdout.write("\x1b[?25l\x1b[2J")
        self.stdout.flush()

    def read_event(self) -> TerminalEvent | None:
        if not self._opened:
            raise RuntimeError("终端尚未打开")
        import os as posix_os
        import select

        ready, _, _ = select.select([self.stdin], [], [], 0)
        if not ready:
            return None
        payload = posix_os.read(self.stdin.fileno(), 1)
        if not payload:
            return EOF_EVENT
        return TerminalEvent.keypress(payload.decode("utf-8", errors="replace"))

    def render(self, snapshot: "RuntimeSnapshot") -> None:
        sample = snapshot.sample
        feedback = snapshot.feedback
        lines = (
            f"模式: {snapshot.mode}    状态: {snapshot.controller_state or '-'}",
            f"高度: {_show(sample.height_mm if sample else None)} mm    raw: {_show(sample.raw_count if sample else None)}",
            f"目标: {_show(snapshot.target_mm)} mm    误差: {_show(snapshot.target_error_mm)} mm",
            f"实际输出: PWM={snapshot.command.lift_pwm} 阀值=0x{snapshot.command.lower_valve:02X} "
            f"加速={snapshot.command.accel} 减速={snapshot.command.decel}",
            f"期望输出: PWM={snapshot.desired_command.lift_pwm} "
            f"阀值=0x{snapshot.desired_command.lower_valve:02X} "
            f"归零请求={'是' if snapshot.zero_requested else '否'}",
            f"泵电流: {_show(feedback.current_raw if feedback else None)}    "
            f"下降电流: {_show(feedback.lower_current_raw if feedback else None)}    "
            f"故障码: {_show(feedback.fault_code if feedback else None)}",
            f"控制故障: {snapshot.controller_fault or '-'}",
            f"CAN泵状态: {snapshot.pump_fault or '-'}",
            f"授权剩余: 起升 {snapshot.lift_remaining_ms} ms / 下降 {snapshot.lower_remaining_ms} ms",
            "操作: u 起升续期700ms | d 下降续期150ms | c 请求清故障 | q 安全退出",
        )
        self.stdout.write("\x1b[H" + "\n".join(lines) + "\x1b[J")
        self.stdout.flush()

    def close(self) -> None:
        if not self._opened:
            return
        import termios

        try:
            termios.tcsetattr(
                self.stdin.fileno(), termios.TCSADRAIN, self._original_attributes
            )
        finally:
            self._opened = False
            self.stdout.write("\x1b[?25h\n")
            self.stdout.flush()


def _show(value: object) -> str:
    return "-" if value is None else str(value)


class DeadmanAuthorizer:
    """按字符事件分别续期起升 700 ms、下降 150 ms 的纯授权器。"""

    LIFT_WINDOW_S = 0.7
    LOWER_WINDOW_S = 0.15

    def __init__(self, *, clock: Callable[[], float] = monotonic) -> None:
        self._clock = clock
        self._lift_until = 0.0
        self._lower_until = 0.0

    def renew_lift(self) -> None:
        self._lift_until = self._clock() + self.LIFT_WINDOW_S

    def renew_lower(self) -> None:
        self._lower_until = self._clock() + self.LOWER_WINDOW_S

    def revoke_all(self) -> None:
        now = self._clock()
        self._lift_until = now
        self._lower_until = now

    @property
    def lift_until(self) -> float:
        return self._lift_until

    @property
    def lower_until(self) -> float:
        return self._lower_until

    @property
    def lift_authorized(self) -> bool:
        return self._clock() < self._lift_until

    @property
    def lower_authorized(self) -> bool:
        return self._clock() < self._lower_until

    @property
    def lift_remaining_ms(self) -> int:
        return _remaining_ms(self._lift_until, self._clock())

    @property
    def lower_remaining_ms(self) -> int:
        return _remaining_ms(self._lower_until, self._clock())


def _remaining_ms(until: float, now: float) -> int:
    """向上取整活动授权，避免二进制浮点把刚续期的 700 ms 显示为 699。"""
    return max(0, ceil(max(0.0, until - now) * 1000.0 - 1e-9))


class ShutdownLatch:
    """信号处理器可安全调用的只写停机闩锁。"""

    def __init__(self) -> None:
        self._event = Event()
        self._lock = Lock()
        self._reason: str | None = None

    def request(self, reason: str) -> None:
        with self._lock:
            if self._reason is None:
                self._reason = str(reason)
        self._event.set()

    @property
    def requested(self) -> bool:
        return self._event.is_set()

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason

    def wait(self, timeout: float | None = None) -> bool:
        return self._event.wait(timeout)


class SensorWorker:
    """独立采样线程；首个打开或读取异常会锁存并暴露给主循环。"""

    def __init__(
        self,
        source: HeightSource,
        *,
        poll_period_s: float,
        sleeper: Callable[[float], None] = sleep,
    ) -> None:
        if type(poll_period_s) not in {int, float} or poll_period_s <= 0:
            raise ValueError("poll_period_s 必须是有限正数")
        self._source = source
        self._period = float(poll_period_s)
        self._sleeper = sleeper
        self._stop = Event()
        self._lock = Lock()
        self._thread: Thread | None = None
        self._latest_sample: HeightSample | None = None
        self._error: str | None = None
        self._closed = False

    @property
    def latest_sample(self) -> HeightSample | None:
        with self._lock:
            return self._latest_sample

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = Thread(target=self._run, name="height-sensor", daemon=True)
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def stop(self) -> None:
        self._stop.set()
        self.join(1.0)
        self._close_once()

    def close(self) -> None:
        self.stop()

    def _run(self) -> None:
        try:
            if self._source.open() is not True:
                raise RuntimeError("高度传感器打开失败")
            while not self._stop.is_set():
                sample = self._source.read_sample()
                if not isinstance(sample, HeightSample):
                    raise TypeError("高度源返回值不是 HeightSample")
                with self._lock:
                    self._latest_sample = sample
                self._sleeper(self._period)
        except Exception as exc:
            with self._lock:
                self._error = f"高度采样线程异常: {exc}"
            self._stop.set()
        finally:
            self._close_once()

    def _close_once(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            self._source.close()
        except Exception as exc:
            with self._lock:
                if self._error is None:
                    self._error = f"关闭高度传感器失败: {exc}"


@dataclass(frozen=True)
class RuntimeSnapshot:
    mode: str
    sample: HeightSample | None = None
    feedback: PumpFeedback | None = None
    target_mm: float | None = None
    target_error_mm: float | None = None
    controller_state: str | None = None
    command: PumpCommand = field(default_factory=PumpCommand.safe_stop)
    desired_command: PumpCommand = field(default_factory=PumpCommand.safe_stop)
    zero_requested: bool = False
    lift_authorized: bool = False
    lower_authorized: bool = False
    lift_remaining_ms: int = 0
    lower_remaining_ms: int = 0
    controller_fault: str | None = None
    pump_fault: str | None = None


CSV_FIELDS = (
    "wall_time",
    "monotonic_s",
    "event",
    "mode",
    "sample_timestamp",
    "sample_raw",
    "sample_height_mm",
    "sample_valid",
    "sample_error",
    "target_mm",
    "target_error_mm",
    "controller_state",
    "command_interlock",
    "command_lift_pwm",
    "command_accel",
    "command_decel",
    "command_lower_valve",
    "desired_interlock",
    "desired_lift_pwm",
    "desired_accel",
    "desired_decel",
    "desired_lower_valve",
    "zero_requested",
    "feedback_timestamp",
    "feedback_current_raw",
    "feedback_fault_code",
    "feedback_lower_current_raw",
    "lift_authorized",
    "lower_authorized",
    "lift_remaining_ms",
    "lower_remaining_ms",
    "controller_fault",
    "pump_fault",
    "operator_key",
    "detail",
)


class CsvEventLogger:
    """逐周期及逐事件写入唯一 CSV；写入和 flush 异常原样上传。"""

    def __init__(
        self,
        log_dir: str | Path,
        mode: str,
        *,
        clock: Callable[[], float] = monotonic,
        wall_clock: Callable[[], float] = time,
    ) -> None:
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = directory / f"{stamp}-{mode}-{uuid4().hex}.csv"
        self._mode = mode
        self._clock = clock
        self._wall_clock = wall_clock
        self._stream: TextIO | None = self.path.open("x", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._stream.flush()

    def log(
        self,
        event: str,
        snapshot: RuntimeSnapshot | None = None,
        *,
        operator_key: str | None = None,
        detail: str | None = None,
    ) -> None:
        if self._stream is None:
            raise RuntimeError("CSV 日志已经关闭")
        value = snapshot or RuntimeSnapshot(self._mode)
        sample = value.sample
        feedback = value.feedback
        command = value.command
        desired = value.desired_command
        row = {
            "wall_time": self._wall_clock(),
            "monotonic_s": self._clock(),
            "event": event,
            "mode": value.mode,
            "sample_timestamp": sample.timestamp if sample else None,
            "sample_raw": sample.raw_count if sample else None,
            "sample_height_mm": sample.height_mm if sample else None,
            "sample_valid": sample.valid if sample else None,
            "sample_error": sample.error if sample else None,
            "target_mm": value.target_mm,
            "target_error_mm": value.target_error_mm,
            "controller_state": value.controller_state,
            "command_interlock": command.interlock,
            "command_lift_pwm": command.lift_pwm,
            "command_accel": command.accel,
            "command_decel": command.decel,
            "command_lower_valve": command.lower_valve,
            "desired_interlock": desired.interlock,
            "desired_lift_pwm": desired.lift_pwm,
            "desired_accel": desired.accel,
            "desired_decel": desired.decel,
            "desired_lower_valve": desired.lower_valve,
            "zero_requested": value.zero_requested,
            "feedback_timestamp": feedback.timestamp if feedback else None,
            "feedback_current_raw": feedback.current_raw if feedback else None,
            "feedback_fault_code": feedback.fault_code if feedback else None,
            "feedback_lower_current_raw": feedback.lower_current_raw if feedback else None,
            "lift_authorized": value.lift_authorized,
            "lower_authorized": value.lower_authorized,
            "lift_remaining_ms": value.lift_remaining_ms,
            "lower_remaining_ms": value.lower_remaining_ms,
            "controller_fault": value.controller_fault,
            "pump_fault": value.pump_fault,
            "operator_key": operator_key,
            "detail": detail,
        }
        self._writer.writerow(row)
        self._stream.flush()

    def close(self) -> None:
        stream = self._stream
        if stream is None:
            return
        self._stream = None
        stream.close()


class _FcntlBackend:
    def acquire(self, stream: TextIO) -> None:
        import fcntl

        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

    def release(self, stream: TextIO) -> None:
        import fcntl

        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


class SingleInstanceLock:
    """state_dir 下的 Linux 非阻塞单实例锁；后端可注入测试。"""

    def __init__(self, path: str | Path, *, backend: Any = None) -> None:
        self.path = Path(path)
        self._backend = backend or _FcntlBackend()
        self._stream: TextIO | None = None

    def acquire(self) -> None:
        if self._stream is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="ascii")
        try:
            self._backend.acquire(stream)
        except (BlockingIOError, OSError) as exc:
            stream.close()
            raise RuntimeError("已有一个运行实例持有单实例锁") from exc
        self._stream = stream

    def release(self) -> None:
        stream = self._stream
        if stream is None:
            return
        self._stream = None
        try:
            self._backend.release(stream)
        finally:
            stream.close()

    def __enter__(self) -> "SingleInstanceLock":
        self.acquire()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.release()
