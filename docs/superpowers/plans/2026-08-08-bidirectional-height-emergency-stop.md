# 双向定高与锁存急停 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让控制器只接收目标高度即可自主选择起升或下降，并提供全局不可运动、解除后不恢复旧目标的锁存软件急停。

**Architecture:** 闭环仍由无硬件依赖的 `HeightController` 生成完整 `PumpCommand`；线程安全的 `EmergencyStopLatch` 同时门控控制器和 CAN 发送层；`LiftHeightControl` 提供未来 `kinco_duolun` 可直接调用的目标与急停接口。当前独立程序继续使用唯一的 `CanPump` 发送线程，后续车辆适配器只需实现相同的命令槽与急停发送回执，不复制闭环状态机。

**Tech Stack:** Python 3.10+、pytest、pymodbus 3.x、python-can 4.x、JSON配置、SocketCAN。

---

## 文件结构与职责

- 新建 `src/agv_lift_height_control/emergency_stop.py`：线程安全急停锁、快照、发送成功/失败证据。
- 新建 `src/agv_lift_height_control/lift_control.py`：公共目标、急停、解除与周期更新门面。
- 修改 `src/agv_lift_height_control/config.py`：增加向后兼容的下降末端参数。
- 修改 `src/agv_lift_height_control/controller.py`：增加自动下降状态机和 `EMERGENCY_STOP` 状态。
- 修改 `src/agv_lift_height_control/can_pump.py`：急停优先级高于期望命令、NMT后持续全零。
- 修改 `src/agv_lift_height_control/application.py`：`move` 使用 `LiftHeightControl` 且不依赖 `u/d`；其他维护模式不变。
- 修改 `src/agv_lift_height_control/operator_runtime.py`：按模式显示操作提示与急停状态，不新增急停按键。
- 修改 `src/agv_lift_height_control/simulation.py`：覆盖下降响应、阀关闭后的延迟与滑行。
- 修改 `src/agv_lift_height_control/__init__.py`：导出稳定公共接口。
- 修改 `config/example.json`、`README.md`、`docs/维护地图.md`：同步配置、行为和未来 `kinco_duolun` 调用链。
- 修改或新增对应 `tests/` 文件：每项生产代码先有失败测试。

### Task 1: 增加向后兼容的下降控制参数

**Files:**
- Modify: `src/agv_lift_height_control/config.py`
- Modify: `config/example.json`
- Test: `tests/test_config.py`

- [ ] **Step 1: 写旧配置默认值和显式配置的失败测试**

在 `tests/test_config.py` 增加：

```python
def test_old_control_config_uses_automatic_lower_defaults(tmp_path) -> None:
    data = valid_config()
    for field in (
        "lower_terminal_zone_mm",
        "lower_pulse_on_s",
        "lower_pulse_wait_s",
    ):
        data["control"].pop(field, None)

    config = load_config(write_config(tmp_path, data))

    assert config.control.lower_terminal_zone_mm == 10.0
    assert config.control.lower_pulse_on_s == 0.05
    assert config.control.lower_pulse_wait_s == 0.70


def test_explicit_automatic_lower_settings_are_loaded(tmp_path) -> None:
    data = valid_config()
    data["control"].update(
        {
            "lower_terminal_zone_mm": 12.0,
            "lower_pulse_on_s": 0.06,
            "lower_pulse_wait_s": 0.80,
        }
    )

    config = load_config(write_config(tmp_path, data))

    assert config.control.lower_terminal_zone_mm == 12.0
    assert config.control.lower_pulse_on_s == 0.06
    assert config.control.lower_pulse_wait_s == 0.80


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("lower_terminal_zone_mm", 2.0),
        ("lower_pulse_on_s", 0.049),
        ("lower_pulse_wait_s", 0.299),
    ],
)
def test_automatic_lower_settings_reject_unsafe_values(tmp_path, field, value) -> None:
    data = valid_config()
    data["control"][field] = value

    with pytest.raises(ConfigError, match=field):
        load_config(write_config(tmp_path, data))
```

- [ ] **Step 2: 运行测试并确认因字段不存在而失败**

Run:

```bash
python -m pytest tests/test_config.py::test_old_control_config_uses_automatic_lower_defaults tests/test_config.py::test_explicit_automatic_lower_settings_are_loaded -q
```

Expected: FAIL，`ControlConfig` 没有 `lower_terminal_zone_mm`。

- [ ] **Step 3: 实现可选默认值和严格范围**

在 `ControlConfig` 末尾增加默认字段：

```python
lower_terminal_zone_mm: float = 10.0
lower_pulse_on_s: float = 0.05
lower_pulse_wait_s: float = 0.70
```

在 `__post_init__` 增加：

```python
_validate_number_range(
    "lower_terminal_zone_mm",
    self.lower_terminal_zone_mm,
    self.tolerance_mm + 0.001,
    50.0,
    section="control",
)
_validate_number_range(
    "lower_pulse_on_s", self.lower_pulse_on_s, 0.05, 0.15, section="control"
)
_validate_number_range(
    "lower_pulse_wait_s", self.lower_pulse_wait_s, 0.3, 1.0, section="control"
)
```

把 `_parse_control` 改为只对三个新字段提供缺省值，其余旧字段仍必须存在：

```python
CONTROL_OPTIONAL_DEFAULTS = {
    "lower_terminal_zone_mm": 10.0,
    "lower_pulse_on_s": 0.05,
    "lower_pulse_wait_s": 0.70,
}


def _parse_control(data: dict[str, Any]) -> ControlConfig:
    _reject_unknown_fields(data, CONTROL_FIELDS, "control 配置")
    values = {}
    for name in CONTROL_FIELDS:
        if name in data:
            values[name] = _number(data, name, positive=True, section="control")
        elif name in CONTROL_OPTIONAL_DEFAULTS:
            values[name] = CONTROL_OPTIONAL_DEFAULTS[name]
        else:
            values[name] = _number(data, name, positive=True, section="control")
    return ControlConfig(**values)
```

在 `config/example.json` 的 `control` 段显式写入三个默认值，并更新 `valid_control_config()` 的示例断言。

- [ ] **Step 4: 运行配置测试并确认通过**

Run: `python -m pytest tests/test_config.py -q`

Expected: PASS。

- [ ] **Step 5: 提交配置改动**

```bash
git add src/agv_lift_height_control/config.py config/example.json tests/test_config.py
git commit -m "feat: configure automatic lowering phases / 配置自动下降阶段参数"
```

### Task 2: 建立线程安全的锁存急停门禁

**Files:**
- Create: `src/agv_lift_height_control/emergency_stop.py`
- Create: `tests/test_emergency_stop.py`
- Modify: `src/agv_lift_height_control/__init__.py`

- [ ] **Step 1: 写锁存、幂等、发送证据和解除条件的失败测试**

新建 `tests/test_emergency_stop.py`：

```python
import pytest

from agv_lift_height_control import EmergencyStopLatch, PumpCommand


class ManualClock:
    def __init__(self, now: float = 10.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def test_emergency_stop_latches_first_reason_and_discards_later_reasons() -> None:
    clock = ManualClock()
    latch = EmergencyStopLatch(clock=clock)

    latch.trigger("operator")
    clock.now = 11.0
    latch.trigger("second caller")

    snapshot = latch.snapshot
    assert snapshot.active is True
    assert snapshot.reason == "operator"
    assert snapshot.triggered_at == 10.0
    assert snapshot.zero_sent_after_trigger is False


def test_emergency_stop_requires_a_post_trigger_zero_send_before_clear() -> None:
    latch = EmergencyStopLatch(clock=ManualClock())
    latch.trigger("operator")

    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()

    latch.record_send_success(PumpCommand.safe_stop())
    latch.clear()

    assert latch.snapshot.active is False


def test_emergency_stop_rejects_nonzero_command_as_clear_evidence() -> None:
    latch = EmergencyStopLatch(clock=ManualClock())
    latch.trigger("operator")
    latch.record_send_success(PumpCommand(interlock=True, lower_valve=0x50))

    with pytest.raises(RuntimeError, match="全零"):
        latch.clear()
```

- [ ] **Step 2: 运行测试并确认导入失败**

Run: `python -m pytest tests/test_emergency_stop.py -q`

Expected: FAIL，缺少 `EmergencyStopLatch`。

- [ ] **Step 3: 实现最小急停锁**

新建 `emergency_stop.py`，实现不可变快照和锁：

```python
from dataclasses import dataclass
from threading import RLock
from time import monotonic
from typing import Callable

from .types import PumpCommand


@dataclass(frozen=True)
class EmergencyStopSnapshot:
    active: bool
    reason: str | None
    triggered_at: float | None
    zero_sent_after_trigger: bool
    transport_fault: str | None


class EmergencyStopLatch:
    def __init__(self, *, clock: Callable[[], float] = monotonic) -> None:
        self._clock = clock
        self._lock = RLock()
        self._active = False
        self._reason = None
        self._triggered_at = None
        self._zero_sent_after_trigger = False
        self._transport_fault = None

    @property
    def snapshot(self) -> EmergencyStopSnapshot:
        with self._lock:
            return EmergencyStopSnapshot(
                self._active,
                self._reason,
                self._triggered_at,
                self._zero_sent_after_trigger,
                self._transport_fault,
            )

    def trigger(self, reason: str) -> None:
        if type(reason) is not str or not reason.strip():
            raise ValueError("急停原因必须是非空字符串")
        with self._lock:
            if self._active:
                return
            self._active = True
            self._reason = reason
            self._triggered_at = self._clock()
            self._zero_sent_after_trigger = False
            self._transport_fault = None

    def record_send_success(self, command: PumpCommand) -> None:
        with self._lock:
            if self._active and command == PumpCommand.safe_stop():
                self._zero_sent_after_trigger = True

    def record_transport_fault(self, reason: str) -> None:
        with self._lock:
            if self._active:
                self._transport_fault = reason

    def record_transport_recovered(self) -> None:
        with self._lock:
            if self._active:
                self._transport_fault = None

    def clear(self) -> None:
        with self._lock:
            if not self._active:
                return
            if not self._zero_sent_after_trigger or self._transport_fault is not None:
                raise RuntimeError("急停解除前必须成功发送全零且传输正常")
            self._active = False
            self._reason = None
            self._triggered_at = None
            self._zero_sent_after_trigger = False
            self._transport_fault = None
```

从 `__init__.py` 导出两个公共类型。

- [ ] **Step 4: 运行急停锁测试并确认通过**

Run: `python -m pytest tests/test_emergency_stop.py -q`

Expected: PASS。

- [ ] **Step 5: 提交急停锁**

```bash
git add src/agv_lift_height_control/emergency_stop.py src/agv_lift_height_control/__init__.py tests/test_emergency_stop.py
git commit -m "feat: add latched emergency stop gate / 增加锁存急停门禁"
```

### Task 3: 在控制器中增加不可运动的急停状态

**Files:**
- Modify: `src/agv_lift_height_control/controller.py`
- Modify: `tests/test_controller.py`

- [ ] **Step 1: 写急停状态优先级的失败测试**

在 `tests/test_controller.py` 增加：

```python
def test_emergency_stop_cancels_target_and_blocks_every_motion_path() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(200.0)
    assert step(controller, 0.0, 100.0).lift_pwm > 0

    controller.enter_emergency_stop("upper layer")

    assert controller.state is ControllerState.EMERGENCY_STOP
    assert controller.target_mm is None
    assert step(controller, 0.05, 100.0) == PumpCommand.safe_stop()
    with pytest.raises(RuntimeError, match="急停"):
        controller.set_target(200.0)
    with pytest.raises(RuntimeError, match="急停"):
        controller.set_manual_lower(True)


def test_cleared_emergency_stop_does_not_restore_old_target() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(200.0)
    controller.enter_emergency_stop("upper layer")

    controller.exit_emergency_stop()

    assert controller.state is ControllerState.MONITOR
    assert controller.target_mm is None
    assert step(controller, 0.1, 100.0) == PumpCommand.safe_stop()
```

- [ ] **Step 2: 运行测试并确认缺少急停状态**

Run: `python -m pytest tests/test_controller.py::test_emergency_stop_cancels_target_and_blocks_every_motion_path tests/test_controller.py::test_cleared_emergency_stop_does_not_restore_old_target -q`

Expected: FAIL，缺少 `ControllerState.EMERGENCY_STOP`。

- [ ] **Step 3: 实现急停状态和最高优先级门禁**

在 `ControllerState` 增加：

```python
EMERGENCY_STOP = "emergency_stop"
```

增加控制器字段与方法：

```python
self.emergency_stop_reason: str | None = None


def enter_emergency_stop(self, reason: str) -> None:
    if type(reason) is not str or not reason.strip():
        raise ValueError("急停原因必须是非空字符串")
    if self.state is ControllerState.EMERGENCY_STOP:
        return
    self.cancel()
    self._external_mode = None
    self.emergency_stop_reason = reason
    self.state = ControllerState.EMERGENCY_STOP
    self._record_actual_command(PumpCommand.safe_stop())


def exit_emergency_stop(self) -> None:
    if self.state is not ControllerState.EMERGENCY_STOP:
        return
    self.emergency_stop_reason = None
    self._target_mm = None
    self._manual_lower = False
    self._reset_terminal_pulse()
    self._record_actual_command(PumpCommand.safe_stop())
    self.state = ControllerState.MONITOR
```

在 `set_target`、`set_manual_lower`、`enter_external_mode` 开头拒绝急停状态。在 `step` 和 `step_external` 的任何输入校验与命令生成之前返回全零，确保急停不会被普通故障清除路径覆盖。Task 4 新增下降相位后，再把 `_reset_lower_pulse()` 加入急停进入和解除路径。

- [ ] **Step 4: 运行控制器急停测试和现有控制器测试**

Run: `python -m pytest tests/test_controller.py -q`

Expected: PASS。

- [ ] **Step 5: 提交控制器急停状态**

```bash
git add src/agv_lift_height_control/controller.py tests/test_controller.py
git commit -m "feat: make emergency stop a motion-blocking state / 将急停设为禁止运动状态"
```

### Task 4: 实现自动下降连续段和末端微脉冲

**Files:**
- Modify: `src/agv_lift_height_control/controller.py`
- Modify: `tests/test_controller.py`

- [ ] **Step 1: 写连续下降与末端相位的失败测试**

在 `tests/test_controller.py` 增加：

```python
def test_target_below_current_height_commands_confirmed_lower_valve() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    command = step(controller, 0.0, 130.0, lift_authorized=False, lower_authorized=False)

    assert controller.state is ControllerState.COARSE_LOWER
    assert command == PumpCommand(interlock=True, lower_valve=0x50)


def test_lower_terminal_zone_settles_then_emits_one_short_pulse() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)

    assert step(controller, 0.00, 108.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.LOWER_SETTLE
    assert step(controller, 0.69, 108.0) == PumpCommand.safe_stop()
    assert step(controller, 0.70, 108.0) == PumpCommand(
        interlock=True, lower_valve=0x50
    )
    assert controller.state is ControllerState.LOWER_PULSE
    assert step(controller, 0.749, 108.0).lower_valve == 0x50
    assert step(controller, 0.75, 107.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.LOWER_SETTLE
    assert step(controller, 1.44, 107.0) == PumpCommand.safe_stop()
    assert step(controller, 1.45, 107.0).lower_valve == 0x50


def test_automatic_command_never_combines_lift_and_lower_outputs() -> None:
    controller = HeightController(control_config(), calibration())
    for target, height in ((200.0, 100.0), (100.0, 200.0), (100.0, 101.0)):
        controller.set_target(target)
        command = step(controller, 0.0, height)
        assert not (command.lift_pwm and command.lower_valve)
```

- [ ] **Step 2: 运行定向测试并确认因仍然只自动起升而失败**

Run: `python -m pytest tests/test_controller.py -k "lower_terminal or target_below or never_combines" -q`

Expected: FAIL，目标低于当前高度仍返回全零/试验失败。

- [ ] **Step 3: 实现下降状态与相位机**

增加状态和内部相位：

```python
COARSE_LOWER = "coarse_lower"
LOWER_SETTLE = "lower_settle"
LOWER_PULSE = "lower_pulse"


class _LowerPulsePhase(str, Enum):
    SETTLE = "settle"
    ON = "on"
```

增加控制器字段：

```python
self._lower_phase: _LowerPulsePhase | None = None
self._lower_phase_started: float | None = None
self._approach_direction: str | None = None
```

把 `_automatic_command` 拆成清晰的方向分派：

```python
def _automatic_command(self, now: float, height_mm: float) -> PumpCommand:
    assert self._target_mm is not None
    error = self._target_mm - height_mm
    if abs(error) <= self.config.tolerance_mm:
        return self._hold_command(now)
    if self._approach_direction == "lower" and error > self.config.tolerance_mm:
        return self._lower_undershoot(now, height_mm, error)
    if self._approach_direction == "lift" and error < -self.config.tolerance_mm:
        return self._lift_overshoot(now, height_mm, -error)
    if error > 0:
        self._approach_direction = "lift"
        self._reset_lower_pulse()
        return self._automatic_lift_command(now, height_mm, error)
    self._approach_direction = "lower"
    self._reset_terminal_pulse()
    return self._automatic_lower_command(now, height_mm, -error)
```

上述辅助函数必须在同一任务定义，不能留下仅转发的空壳：

```python
def _hold_command(self, now: float) -> PumpCommand:
    self._reset_terminal_pulse()
    self._reset_lower_pulse()
    if self._stable_since is None:
        self._stable_since = now
    if now - self._stable_since + 1e-12 >= self.config.stable_time_s:
        self.state = ControllerState.HOLD
        self._approach_direction = None
    else:
        self.state = ControllerState.IDLE
    return PumpCommand.hydraulic_hold()


def _lower_undershoot(
    self, now: float, height_mm: float, undershoot_mm: float
) -> PumpCommand:
    if undershoot_mm > self.config.overshoot_limit_mm:
        return self._fault(
            f"目标下冲 {undershoot_mm:.3f} mm，超过安全上限",
            kind="undershoot",
            height_mm=height_mm,
            timestamp=now,
        )
    self.state = ControllerState.IDLE
    self.trial_failed = True
    self._stable_since = None
    self._reset_lower_pulse()
    return PumpCommand.safe_stop()


def _lift_overshoot(
    self, now: float, height_mm: float, overshoot_mm: float
) -> PumpCommand:
    if overshoot_mm > self.config.overshoot_limit_mm:
        return self._fault(
            f"目标超调 {overshoot_mm:.3f} mm，超过安全上限",
            kind="overshoot",
            height_mm=height_mm,
            timestamp=now,
        )
    self.state = ControllerState.IDLE
    self.trial_failed = True
    self._stable_since = None
    self._reset_terminal_pulse()
    return PumpCommand.safe_stop()
```

`_automatic_lift_command(now, height_mm, error)` 必须移动当前 `_automatic_command` 中“清除稳定计时→继续已有末端相位→粗升→P调速→末端脉冲”的完整代码，参数 `error` 直接替代原局部误差，行为和量化规则保持不变：

```python
def _automatic_lift_command(
    self, now: float, height_mm: float, error: float
) -> PumpCommand:
    self._stable_since = None
    if self._terminal_pulse_phase is not None:
        return self._terminal_pulse_command(now, height_mm)
    if error > self.slow_zone_mm:
        self.state = ControllerState.COARSE_LIFT
        return self._lift_command(self.calibration.coarse_pwm, height_mm)
    if error > self.pulse_zone_mm:
        self.state = ControllerState.P_CONTROL
        scale = (error - self.pulse_zone_mm) / (
            self.slow_zone_mm - self.pulse_zone_mm
        )
        raw_pwm = self.calibration.min_stable_pwm + scale * (
            self.calibration.coarse_pwm - self.calibration.min_stable_pwm
        )
        levels = (
            value
            for value in LIFT_PWM_LEVELS
            if self.calibration.min_stable_pwm
            <= value
            <= self.calibration.coarse_pwm
            and value >= raw_pwm
        )
        pwm = next(levels, self.calibration.coarse_pwm)
        return self._lift_command(pwm, height_mm)
    return self._terminal_pulse_command(now, height_mm)
```

下降命令保持独立：

```python
def _automatic_lower_command(
    self, now: float, height_mm: float, distance_mm: float
) -> PumpCommand:
    self._stable_since = None
    if self._lower_phase is not None:
        return self._lower_terminal_command(now)
    if distance_mm > self.config.lower_terminal_zone_mm:
        self.state = ControllerState.COARSE_LOWER
        return PumpCommand(
            interlock=True,
            lower_valve=self.calibration.lower_comfortable_valve,
        )
    self._lower_phase = _LowerPulsePhase.SETTLE
    self._lower_phase_started = now
    self.state = ControllerState.LOWER_SETTLE
    return PumpCommand.safe_stop()
```

`_lower_terminal_command` 在 `SETTLE` 满 `lower_pulse_wait_s` 后转 `ON`，在 `ON` 满 `lower_pulse_on_s` 后立即全零并回到 `SETTLE`。`_reset_lower_pulse` 清空两个内部字段。所有 `set_target/cancel/fault/hold/急停` 路径都调用该复位函数。

下冲处理必须以当前接近方向为前提：超过容差设置 `trial_failed=True` 并保持全零；超过 `overshoot_limit_mm` 调用 `_fault(..., kind="undershoot")`。

- [ ] **Step 4: 增加Hold后漂移与下冲边界测试**

```python
def test_lower_undershoot_beyond_two_stops_and_beyond_five_faults() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)
    assert step(controller, 0.0, 120.0).lower_valve == 0x50

    assert step(controller, 0.02, 97.9) == PumpCommand.safe_stop()
    assert controller.trial_failed is True

    controller.set_target(100.0)
    assert step(controller, 1.0, 120.0).lower_valve == 0x50
    assert step(controller, 1.02, 94.9) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT


def test_hold_drift_can_start_a_new_automatic_correction() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)
    for now in (0.0, 0.5):
        assert step(controller, now, 100.0) == PumpCommand.hydraulic_hold()
    assert controller.state is ControllerState.HOLD

    assert step(controller, 0.52, 103.0) == PumpCommand.safe_stop()
    assert controller.state is ControllerState.LOWER_SETTLE
```

- [ ] **Step 5: 运行控制器全量测试**

Run: `python -m pytest tests/test_controller.py -q`

Expected: PASS。原“起升授权丢失后重启脉冲”测试先保留，自动模式去授权的改变放到 Task 6，避免一次提交混合两个行为。

- [ ] **Step 6: 提交自动下降核心**

```bash
git add src/agv_lift_height_control/controller.py tests/test_controller.py
git commit -m "feat: add automatic lowering state machine / 增加自动下降状态机"
```

### Task 5: 让CAN发送层独立执行急停全零

**Files:**
- Modify: `src/agv_lift_height_control/can_pump.py`
- Modify: `tests/test_can_pump.py`

- [ ] **Step 1: 写急停覆盖非零期望命令的失败测试**

在 `tests/test_can_pump.py` 增加：

```python
def test_emergency_stop_overrides_nonzero_desired_command_within_one_cycle() -> None:
    clock = ManualClock(0.0)
    latch = EmergencyStopLatch(clock=clock)
    bus = FakeBus()
    pump, _, sleeper = make_pump(bus, clock=clock, emergency_stop=latch)
    pump.start()
    try:
        clock.advance(5.0)
        pump._commit_feedback(PumpFeedback(clock(), 0, 0, 0))
        pump.update_command(PumpCommand(interlock=True, lower_valve=0x50))
        assert pump.run_cycle(clock()).lower_valve == 0x50

        latch.trigger("upper layer")
        clock.advance(0.05)

        assert pump.run_cycle(clock()) == PumpCommand.safe_stop()
        assert "急停" in (pump.fault_reason or "")
    finally:
        sleeper.release.set()
        pump.stop()


def test_nonzero_updates_during_emergency_stop_cannot_resume_after_clear() -> None:
    clock = ManualClock(0.0)
    latch = EmergencyStopLatch(clock=clock)
    bus = FakeBus()
    pump, _, sleeper = make_pump(bus, clock=clock, emergency_stop=latch)
    pump.start()
    try:
        latch.trigger("upper layer")
        pump.update_command(PumpCommand(interlock=True, lift_pwm=40))
        pump.run_cycle(clock())
        latch.clear()

        assert pump.desired_command == PumpCommand.safe_stop()
    finally:
        sleeper.release.set()
        pump.stop()
```

- [ ] **Step 2: 运行测试并确认构造参数或门禁缺失**

Run: `python -m pytest tests/test_can_pump.py -k emergency_stop -q`

Expected: FAIL，`CanPump` 不接受 `emergency_stop`。

- [ ] **Step 3: 注入急停锁并放到最高优先级**

先给现有测试辅助函数 `make_pump` 增加 `emergency_stop=None` 关键字并原样传给构造器。`CanPump.__init__` 增加：

```python
emergency_stop: EmergencyStopLatch | None = None
```

保存 `self.emergency_stop = emergency_stop or EmergencyStopLatch(clock=clock)`。在周期选择命令的最前面增加：

```python
snapshot = self.emergency_stop.snapshot
if snapshot.active:
    return PumpCommand.safe_stop(), f"急停锁存: {snapshot.reason}"
```

`update_command` 在急停时把期望槽保持为严格全零，不能缓存非零命令。每次成功发送后调用：

```python
self.emergency_stop.record_send_success(command)
```

发送异常时调用：

```python
self.emergency_stop.record_transport_fault(str(exc))
```

停止阶段的补发零帧也记录成功证据。只有完成明确的CAN重启/恢复门禁后才调用 `record_transport_recovered()`；普通发送成功不得静默擦除已经锁存的传输异常。

- [ ] **Step 4: 验证50 ms门禁、NMT和退出回归**

Run: `python -m pytest tests/test_can_pump.py -q`

Expected: PASS，包括现有 NMT、反馈超时、50 ms调度和退出补零测试。

- [ ] **Step 5: 提交CAN急停门禁**

```bash
git add src/agv_lift_height_control/can_pump.py tests/test_can_pump.py
git commit -m "feat: force CAN zero while emergency stop is latched / 急停锁存时强制CAN全零"
```

### Task 6: 增加公共控制门面并让move自主运行

**Files:**
- Create: `src/agv_lift_height_control/lift_control.py`
- Create: `tests/test_lift_control.py`
- Modify: `src/agv_lift_height_control/application.py`
- Modify: `src/agv_lift_height_control/operator_runtime.py`
- Modify: `src/agv_lift_height_control/__init__.py`
- Modify: `tests/test_application_modes.py`
- Modify: `tests/test_operator_runtime.py`
- Modify: `tests/test_controller.py`

- [ ] **Step 1: 写门面目标、急停和解除健康检查的失败测试**

新建 `tests/test_lift_control.py`：

```python
import pytest

from agv_lift_height_control import (
    CalibrationBundle,
    ControlConfig,
    ControllerState,
    EmergencyStopLatch,
    HeightController,
    HeightSample,
    LiftHeightControl,
    PumpCommand,
    PumpFeedback,
)


class ManualClock:
    def __init__(self, now: float = 10.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def make_controller() -> HeightController:
    config = ControlConfig(
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
    )
    calibration = CalibrationBundle(
        min_stable_pwm=40,
        coarse_pwm=40,
        response_delay_s=0.15,
        max_coast_mm=4.3,
        peak_current_by_pwm={40: 1112},
        lower_min_start_valve=0x50,
        lower_comfortable_valve=0x50,
        soft_upper_limit_mm=950.0,
    )
    return HeightController(config, calibration)


def height_sample(now: float, height: float) -> HeightSample:
    return HeightSample(now, round(height * 20.48), height, True, None)


def pump_feedback(now: float, *, fault: int = 0) -> PumpFeedback:
    return PumpFeedback(now, 0, fault, 0)


def test_emergency_stop_blocks_targets_and_clear_never_restores_old_target() -> None:
    clock = ManualClock()
    latch = EmergencyStopLatch(clock=clock)
    controller = make_controller()
    control = LiftHeightControl(controller, latch, clock=clock)
    control.set_target_height(200.0)

    control.emergency_stop("PLC emergency")
    with pytest.raises(RuntimeError, match="急停"):
        control.set_target_height(300.0)

    latch.record_send_success(PumpCommand.safe_stop())
    control.update(clock(), height_sample(clock(), 100.0), pump_feedback(clock()))
    control.clear_emergency_stop()

    assert controller.target_mm is None
    assert controller.state is ControllerState.MONITOR


def test_clear_emergency_stop_rejects_stale_feedback() -> None:
    clock = ManualClock(now=10.0)
    latch = EmergencyStopLatch(clock=clock)
    control = LiftHeightControl(
        make_controller(), latch, clock=clock
    )
    control.emergency_stop("PLC emergency")
    latch.record_send_success(PumpCommand.safe_stop())
    control.update(10.0, height_sample(10.0, 100.0), pump_feedback(9.0))

    with pytest.raises(RuntimeError, match="反馈"):
        control.clear_emergency_stop()
```

- [ ] **Step 2: 运行测试并确认公共门面缺失**

Run: `python -m pytest tests/test_lift_control.py -q`

Expected: FAIL，缺少 `LiftHeightControl`。

- [ ] **Step 3: 实现线程安全门面**

新建 `lift_control.py`：

```python
class LiftHeightControl:
    def __init__(self, controller, emergency_stop, *, clock=monotonic) -> None:
        self.controller = controller
        self.emergency_stop_latch = emergency_stop
        self._clock = clock
        self._lock = RLock()
        self._last_sample = None
        self._last_feedback = None

    def set_target_height(self, target_mm, *, temporary_max_height_mm=None) -> None:
        with self._lock:
            if self.emergency_stop_latch.snapshot.active:
                raise RuntimeError("急停状态禁止设置目标")
            self.controller.set_target(
                target_mm,
                temporary_max_height_mm=temporary_max_height_mm,
            )

    def emergency_stop(self, reason: str) -> None:
        self.emergency_stop_latch.trigger(reason)
        with self._lock:
            self.controller.enter_emergency_stop(reason)

    def update(self, now, sample, feedback) -> PumpCommand:
        with self._lock:
            self._last_sample = sample
            self._last_feedback = feedback
            if self.emergency_stop_latch.snapshot.active:
                self.controller.enter_emergency_stop(
                    self.emergency_stop_latch.snapshot.reason or "unknown"
                )
            return self.controller.step(
                now=now,
                sample=sample,
                feedback=feedback,
                lift_authorized=True,
                lower_authorized=True,
            )

    def clear_emergency_stop(self) -> None:
        with self._lock:
            self._validate_clear_inputs(self._clock())
            self.emergency_stop_latch.clear()
            self.controller.exit_emergency_stop()
```

`_validate_clear_inputs` 严格检查最近样本和反馈存在、时间戳不在未来、分别未超过控制配置的100/150 ms、反馈故障码为0，并依赖锁的 `clear()` 检查急停后全零发送证据。

从包根导出 `LiftHeightControl`。

- [ ] **Step 4: 让MoveCommandSource使用门面且忽略键盘运动授权**

把 `MoveCommandSource` 改为：

```python
class MoveCommandSource:
    allow_lift = False
    allow_lower = False

    def __init__(self, control: LiftHeightControl) -> None:
        self.control = control
        self.controller = control.controller

    def step(self, now, sample, feedback, lift_authorized, lower_authorized):
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        return CommandDecision(self.control.update(now, sample, feedback))
```

在 `_run_mode` 创建一个共享 `EmergencyStopLatch`，传给 `_build_control_source` 和 `pump_factory`。将 `ApplicationDependencies.pump_factory` 统一改为接收 `(can_config, emergency_stop)`，同步更新所有测试假工厂。

`_build_control_source` 的 `move` 分支创建门面后再设置目标：

```python
control = LiftHeightControl(controller, emergency_stop, clock=clock)
control.set_target_height(
    args.target_mm,
    temporary_max_height_mm=args.temporary_max_mm,
)
return MoveCommandSource(control)
```

- [ ] **Step 5: 更新运行时按键边界测试**

增加应用测试，证明 `move` 在授权均为False时仍产生起升或下降命令；同时证明 `manual-lower`、标定和survey的授权测试保持不变。把原 `test_terminal_authorization_loss_restarts_with_full_settle` 改成：

```python
def test_automatic_move_does_not_depend_on_deadman_authorization() -> None:
    controller = HeightController(control_config(), calibration())
    control = LiftHeightControl(controller, EmergencyStopLatch())
    control.set_target_height(110.0)

    for now in (0.0, 0.65):
        command = control.update(now, sample(now, 100.0), feedback(now))

    assert command.lift_pwm == 50
```

TUI根据模式显示：`move` 只显示“自主运动中 | q安全退出”，不显示 `u/d/c`；维护模式保持原提示。不得增加 `e` 键。

`MoveCommandSource` 暴露：

```python
self.emergency_stop_latch = control.emergency_stop_latch
```

在 `RuntimeSnapshot` 和CSV增加独立字段，不能把急停伪装成普通控制故障：

```python
emergency_stop_active: bool = False
emergency_stop_reason: str | None = None
```

`ForegroundRuntime._snapshot()` 从 `source.emergency_stop_latch.snapshot` 读取这两个值；`CsvEventLogger.CSV_FIELDS` 和行映射增加同名列；TUI显示“急停状态/原因”。在 `tests/test_operator_runtime.py` 增加CSV和界面断言：锁存时两列分别为 `True` 和第一次触发原因，普通运行时为 `False` 和空值。

- [ ] **Step 6: 运行门面、应用和运行时测试**

Run:

```bash
python -m pytest tests/test_lift_control.py tests/test_application_modes.py tests/test_operator_runtime.py tests/test_foreground_loop.py -q
```

Expected: PASS。

- [ ] **Step 7: 提交公共接口与自主move**

```bash
git add src/agv_lift_height_control/lift_control.py src/agv_lift_height_control/application.py src/agv_lift_height_control/operator_runtime.py src/agv_lift_height_control/__init__.py tests/test_lift_control.py tests/test_application_modes.py tests/test_operator_runtime.py tests/test_controller.py
git commit -m "feat: expose autonomous bidirectional height control / 暴露自主双向定高接口"
```

### Task 7: 用现有液压仿真增加集成回归验证

**Files:**
- Modify: `tests/test_simulation.py`

- [ ] **Step 1: 增加带下降延迟和停阀滑行的集成测试**

Task 4 的单元测试已经在生产实现前完成RED；本任务不驱动新的生产行为，只增加跨组件回归验证。在 `tests/test_simulation.py` 的公共导入中加入 `EmergencyStopLatch` 和 `LiftHeightControl`，再增加：

```python
def test_controller_reaches_lower_target_with_delayed_hydraulics() -> None:
    simulator = HydraulicLiftSimulator(
        initial_height_mm=200.0,
        min_lift_pwm=35,
        response_delay_s=0.15,
        max_lift_speed_mm_s=220.0,
        max_lower_speed_mm_s=90.0,
        coast_decay_s=0.08,
        fixed_step_s=0.05,
    )
    controller = HeightController(
        control_config(), calibration(coast=4.3, response=0.15)
    )
    control = LiftHeightControl(controller, EmergencyStopLatch())
    control.set_target_height(80.0)
    minimum = simulator.height_mm
    nonzero_lower_commands = 0

    for _ in range(1200):
        observed = simulator.observe()
        command = control.update(
            observed.now,
            observed.sample,
            observed.feedback,
        )
        nonzero_lower_commands += int(command.lower_valve > 0)
        observed = simulator.advance(command)
        minimum = min(minimum, observed.height_mm)
        if controller.state is ControllerState.HOLD:
            break

    assert controller.state is ControllerState.HOLD, controller.fault_reason
    assert nonzero_lower_commands > 0
    assert minimum >= 75.0
    assert 78.0 <= simulator.height_mm <= 82.0


def test_simulated_emergency_stop_keeps_all_future_commands_zero() -> None:
    simulator = HydraulicLiftSimulator(
        initial_height_mm=200.0,
        response_delay_s=0.15,
        fixed_step_s=0.05,
    )
    controller = HeightController(
        control_config(), calibration(coast=4.3, response=0.15)
    )
    latch = EmergencyStopLatch()
    control = LiftHeightControl(controller, latch)
    control.set_target_height(80.0)

    observed = simulator.observe()
    simulator.advance(control.update(observed.now, observed.sample, observed.feedback))
    control.emergency_stop("test")

    commands = []
    for _ in range(40):
        observed = simulator.observe()
        command = control.update(observed.now, observed.sample, observed.feedback)
        commands.append(command)
        simulator.advance(command)

    assert set(commands) == {PumpCommand.safe_stop()}
```

- [ ] **Step 2: 运行新增集成测试并确认通过**

Run: `python -m pytest tests/test_simulation.py -k "lower_target or emergency_stop" -q`

Expected: PASS，并且第一项测试统计到至少一次非零下降阀命令。

- [ ] **Step 3: 审查并保留现有仿真模型**

确认 `HydraulicLiftSimulator._command_target_velocity()` 已把下降阀转换为负目标速度，并与起升共用 `_velocity_history` 延迟历史和 `_integrate_coast()` 指数滑行；本任务不得重写该模型。若确定性参数使50 ms脉冲量化后下冲超过5 mm，只允许调整测试液压模型的 `max_lower_speed_mm_s`/`coast_decay_s` 到与现场 `0x50` 150 ms约2.637 mm一致；不得放宽控制器±2/5 mm安全断言。

- [ ] **Step 4: 运行全部仿真与控制器测试**

Run: `python -m pytest tests/test_simulation.py tests/test_controller.py -q`

Expected: PASS。

- [ ] **Step 5: 提交仿真**

```bash
git add tests/test_simulation.py
git commit -m "test: simulate delayed automatic lowering / 仿真延迟自动下降"
```

### Task 8: 更新README、维护地图和集成说明

**Files:**
- Modify: `README.md`
- Modify: `docs/维护地图.md`
- Modify: `docs/外部命令安全仲裁.md`
- Test: `tests/test_readme_commands.py`

- [ ] **Step 1: 写文档契约失败测试**

在 `tests/test_readme_commands.py` 增加：

```python
def test_readme_describes_autonomous_move_and_latched_emergency_stop() -> None:
    text = Path("README.md").read_text(encoding="utf-8")

    assert "move 模式不需要持续按 u 或 d" in text
    assert "EMERGENCY_STOP" in text
    assert "解除急停后必须重新下发目标" in text
    assert "0x217 唯一发送者" in text
```

- [ ] **Step 2: 运行测试并确认旧文档描述不匹配**

Run: `python -m pytest tests/test_readme_commands.py::test_readme_describes_autonomous_move_and_latched_emergency_stop -q`

Expected: FAIL，README仍描述 `move` 持续按 `u`。

- [ ] **Step 3: 更新文档和维护追踪**

README必须明确：

- `move` 是自主运动命令，给目标后自动选择方向；
- 标定、survey和manual-lower仍需原死手授权；
- 急停只提供函数，不增加键盘按键；
- 急停状态不可运动，解除后无目标；
- 当前软件急停车辆上层触发链尚未实机验证；
- 未来集成必须删除/替换 `kinco_duolun` 固定全零 `0x217`，不能新增第二发送线程。

维护地图补充变量映射：

```text
lower_terminal_zone_mm → HeightController._automatic_lower_command → tests/test_controller.py
lower_pulse_on_s       → HeightController._lower_terminal_command   → tests/test_controller.py
lower_pulse_wait_s     → HeightController._lower_terminal_command   → tests/test_controller.py
EmergencyStopLatch     → CanPump.run_cycle/LiftHeightControl → 急停测试
```

函数地图补充 `set_target_height/emergency_stop/clear_emergency_stop/update` 以及未来 `kinco_duolun` 的单一命令槽调用链。

- [ ] **Step 4: 运行文档测试并检查Markdown**

Run: `python -m pytest tests/test_readme_commands.py -q`

Expected: PASS。

Run: `git diff --check`

Expected: 无输出，退出码0。

- [ ] **Step 5: 提交文档**

```bash
git add README.md docs/维护地图.md docs/外部命令安全仲裁.md tests/test_readme_commands.py
git commit -m "docs: document autonomous lift safety boundaries / 记录自主升降安全边界"
```

### Task 9: 全量验证、接口审查与交付提交

**Files:**
- Verify: `src/agv_lift_height_control/`
- Verify: `tests/`
- Verify: `config/example.json`
- Verify: `README.md`
- Verify: `docs/维护地图.md`

- [ ] **Step 1: 运行全量测试**

Run: `python -m pytest -q`

Expected: 全部PASS，无warning、error或跳过的新增测试。

- [ ] **Step 2: 运行编译检查**

Run: `python -m compileall -q src tests`

Expected: 无输出，退出码0。

- [ ] **Step 3: 检查差异和公共接口**

```bash
git diff --check
git status --short
python - <<'PY'
from agv_lift_height_control import (
    EmergencyStopLatch,
    LiftHeightControl,
)
print(EmergencyStopLatch.__name__, LiftHeightControl.__name__)
PY
```

Expected: 差异检查无输出；只存在本计划范围内文件；两个公共类型可导入。

- [ ] **Step 4: 对照设计逐项审查**

逐项确认：

- 自动目标不读取 `u/d`；
- 维护模式仍读取原死手授权；
- 下降只使用最终标定的舒适阀值；
- 急停锁优先于所有命令；
- CAN发送成功证据发生在急停触发之后；
- 解除后目标为None且命令全零；
- 当前和未来适配都只允许一个 `0x217`发送者；
- 未做的车辆上层急停实机链在README中明确。

- [ ] **Step 5: 如有未提交的同范围收尾改动，创建双语提交**

```bash
git add src tests config README.md docs
git diff --cached --check
git commit -m "feat: complete bidirectional height and emergency stop / 完成双向定高与急停"
```

如果工作区已经干净，则跳过空提交。

- [ ] **Step 6: 记录Orange Pi最小冒烟边界**

交付说明只安排一次低高度自动下降：从已确认安全高度下降到一个非零低目标，现场人员可物理断电；不重复起升定高。公共急停函数在尚未接入 `kinco_duolun` 前只报告自动化验证结果，不宣称车辆上层触发链已经实机通过。
