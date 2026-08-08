# 末端先停泵与液压保持实施计划

> **面向代理执行者：** 必须使用 `executing-plans` 或 `subagent-driven-development` 逐项实施；所有生产代码改动必须先有按预期失败的测试，并以复选框跟踪步骤。

**目标：** 修复连续起升直接衔接末端脉冲造成的现场超调，并在目标稳定窗口内发送 `217#0100000000000000` 保持液压互锁，同时保证全部故障和退出路径继续严格全零。

**架构：** `HeightController` 使用独立内部相位表示首次停泵观察、通泵和脉冲后观察；`PumpCommand` 用两个命名构造方法区分“液压保持”和“安全停机”。协议编码和 `CanPump` 安全门禁不改变，TUI只增加互锁可见性。

**技术栈：** Python 3.10+、pytest、现有纯状态控制器、python-can 4.x、SSH ANSI TUI。

---

## 文件结构与职责

- 修改 `src/agv_lift_height_control/types.py`：增加正常液压保持命令，严格保留 `safe_stop()` 全零语义。
- 修改 `src/agv_lift_height_control/controller.py`：增加末端内部相位和转换，目标稳定窗口返回液压保持命令。
- 修改 `src/agv_lift_height_control/operator_runtime.py`：显示实际与期望互锁状态。
- 修改 `tests/test_types.py`：验证两个零输出命令的语义差异。
- 修改 `tests/test_can_pump.py`：验证 Byte0 编码与安全全零编码都不回退。
- 修改 `tests/test_controller.py`：覆盖现场末端切换、脉冲时序、授权中断、保持和故障全零。
- 修改 `tests/test_operator_runtime.py`：验证互锁状态进入 SSH TUI。
- 修改 `README.md`：说明闭环末端与保持帧。
- 修改 `docs/维护地图.md`：更新变量、函数、状态和测试映射。

不修改 Modbus、标定草稿、最终标定 JSON、CAN 发送周期和反馈协议。

### 任务1：区分液压保持与安全停机命令

**文件：**

- 修改：`tests/test_types.py`
- 修改：`tests/test_can_pump.py`
- 修改：`src/agv_lift_height_control/types.py`

- [ ] **步骤1：先写命令语义失败测试**

在 `tests/test_types.py` 增加：

```python
def test_hydraulic_hold_enables_only_interlock_and_safe_stop_remains_all_zero() -> None:
    assert PumpCommand.hydraulic_hold() == PumpCommand(True, 0, 0, 0, 0)
    assert PumpCommand.safe_stop() == PumpCommand(False, 0, 0, 0, 0)
```

在 `tests/test_can_pump.py::test_encode_command_and_nmt_payloads_follow_protocol_layout` 增加：

```python
assert encode_command(PumpCommand.hydraulic_hold()) == bytes([1, 0, 0, 0, 0, 0, 0, 0])
assert encode_command(PumpCommand.safe_stop()) == bytes(8)
```

- [ ] **步骤2：运行测试并确认RED**

运行：

```powershell
py -3.13 -m pytest tests/test_types.py tests/test_can_pump.py::test_encode_command_and_nmt_payloads_follow_protocol_layout -q
```

预期：失败于 `AttributeError: type object 'PumpCommand' has no attribute 'hydraulic_hold'`，证明测试命中了缺失行为。

- [ ] **步骤3：实现最小命名构造方法**

在 `PumpCommand.safe_stop()` 前增加：

```python
@classmethod
def hydraulic_hold(cls) -> "PumpCommand":
    """返回只启用互锁的正常保持命令；不得用于故障、超时或退出。"""
    return cls(interlock=True)
```

保持 `safe_stop()` 实现仍为：

```python
@classmethod
def safe_stop(cls) -> "PumpCommand":
    """返回不使能且所有输出为零的安全停机命令。"""
    return cls()
```

- [ ] **步骤4：运行测试并确认GREEN**

运行步骤2的同一命令，预期全部通过。

- [ ] **步骤5：提交命令类型改动**

```powershell
git add -- tests/test_types.py tests/test_can_pump.py src/agv_lift_height_control/types.py
git commit -m "feat: distinguish hydraulic hold from safe stop / 区分液压保持与安全停机"
```

### 任务2：进入末端区时先完成停泵观察

**文件：**

- 修改：`tests/test_controller.py`
- 修改：`src/agv_lift_height_control/controller.py`

- [ ] **步骤1：把现场时序写成失败回归测试**

在 `tests/test_controller.py` 增加：

```python
def test_field_transition_from_continuous_lift_stops_before_terminal_pulse() -> None:
    controller = HeightController(control_config(), single_level_calibration())
    controller.set_target(80.0)

    moving = step(controller, 0.0, 46.337890625)
    assert controller.state is ControllerState.P_CONTROL
    assert moving == PumpCommand(interlock=True, lift_pwm=40)

    terminal_entry = step(controller, 0.513, 68.505859375)
    assert controller.state is ControllerState.TERMINAL_PULSE
    assert terminal_entry == PumpCommand.safe_stop()
```

修改 `test_control_zones_derive_from_coast_and_use_deterministic_boundaries`，把末端边界首周期断言改为：

```python
assert pulse == PumpCommand.safe_stop()
```

把单档标定用例改为明确区分连续区和末端首次停泵：

```python
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
```

把 `test_terminal_pulse_uses_clamped_response_and_wait_phases` 改为：

```python
def test_terminal_pulse_settles_before_first_on_and_waits_after_each_pulse() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(110.0)

    assert controller.pulse_on_s == pytest.approx(0.15)
    assert controller.pulse_wait_s == pytest.approx(0.65)
    assert step(controller, 0.0, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.64, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.65, 100.0).lift_pwm == 50
    assert step(controller, 0.79, 100.0).lift_pwm == 50
    assert step(controller, 0.80, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.44, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.45, 100.0).lift_pwm == 50
```

并增加授权中断回归：

```python
def test_terminal_authorization_loss_restarts_with_full_settle() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(110.0)

    assert step(controller, 0.0, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 0.65, 100.0).lift_pwm == 50
    assert step(controller, 0.70, 100.0, lift_authorized=False) == PumpCommand.safe_stop()
    assert step(controller, 0.71, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.35, 100.0) == PumpCommand.safe_stop()
    assert step(controller, 1.36, 100.0).lift_pwm == 50
```

- [ ] **步骤2：运行现场回归测试并确认RED**

运行：

```powershell
py -3.13 -m pytest tests/test_controller.py::test_field_transition_from_continuous_lift_stops_before_terminal_pulse tests/test_controller.py::test_control_zones_derive_from_coast_and_use_deterministic_boundaries tests/test_controller.py::test_terminal_pulse_settles_before_first_on_and_waits_after_each_pulse tests/test_controller.py::test_terminal_authorization_loss_restarts_with_full_settle -q
```

预期：现有实现进入末端区就返回最低稳定 PWM，首次停泵与授权中断后的完整停泵观察断言失败。

- [ ] **步骤3：增加显式末端相位和统一重置方法**

在 `ControllerState` 后增加私有枚举：

```python
class _TerminalPulsePhase(str, Enum):
    SETTLE = "settle"
    ON = "on"
    WAIT = "wait"
```

在 `HeightController.__init__()` 增加：

```python
self._terminal_pulse_phase: _TerminalPulsePhase | None = None
```

增加统一清理方法，并用它替换自动目标、取消目标、外部模式、故障、控制分区和起升授权门控中零散的 `_pulse_phase_started = None`：

```python
def _reset_terminal_pulse(self) -> None:
    self._terminal_pulse_phase = None
    self._pulse_phase_started = None
```

- [ ] **步骤4：实现首次SETTLE、ON和WAIT转换**

增加：

```python
def _terminal_pulse_command(self, now: float, height_mm: float) -> PumpCommand:
    self.state = ControllerState.TERMINAL_PULSE
    if self._terminal_pulse_phase is None or self._pulse_phase_started is None:
        # 从连续运动进入末端区时先停泵，不能把进入时刻当作新脉冲起点。
        self._terminal_pulse_phase = _TerminalPulsePhase.SETTLE
        self._pulse_phase_started = now
        return PumpCommand.safe_stop()

    elapsed = now - self._pulse_phase_started
    if self._terminal_pulse_phase in {
        _TerminalPulsePhase.SETTLE,
        _TerminalPulsePhase.WAIT,
    }:
        if elapsed + 1e-12 < self.pulse_wait_s:
            return PumpCommand.safe_stop()
        self._terminal_pulse_phase = _TerminalPulsePhase.ON
        self._pulse_phase_started = now
        return self._lift_command(self.calibration.min_stable_pwm, height_mm)

    if elapsed + 1e-12 < self.pulse_on_s:
        return self._lift_command(self.calibration.min_stable_pwm, height_mm)
    self._terminal_pulse_phase = _TerminalPulsePhase.WAIT
    self._pulse_phase_started = now
    return PumpCommand.safe_stop()
```

把 `_automatic_command()` 原末端计时分支替换为：

```python
return self._terminal_pulse_command(now, height_mm)
```

在 `HeightController.step()` 的起升授权门控中使用统一重置：

```python
if command.lift_pwm and not lift_authorized:
    # 通泵中撤权立即全零；再次授权必须重新完成停泵观察。
    if self.state is ControllerState.TERMINAL_PULSE:
        self._reset_terminal_pulse()
    command = safe_stop
```

- [ ] **步骤5：运行现场回归测试并确认GREEN**

重新运行步骤2，预期四个测试通过。

- [ ] **步骤6：运行控制器定向测试并确认GREEN**

运行：

```powershell
py -3.13 -m pytest tests/test_controller.py -q
```

预期：控制器测试全部通过，故障和限位断言没有放宽。

- [ ] **步骤7：提交末端状态机改动**

```powershell
git add -- tests/test_controller.py src/agv_lift_height_control/controller.py
git commit -m "fix: settle before terminal lift pulses / 末端起升脉冲前先停泵观察"
```

### 任务3：目标稳定窗口启用液压保持

**文件：**

- 修改：`tests/test_controller.py`
- 修改：`src/agv_lift_height_control/controller.py`

- [ ] **步骤1：先写保持与故障全零失败测试**

把 `test_target_requires_500ms_continuous_tolerance_before_hold` 的核心断言改为：

```python
for now in (0.0, 0.1, 0.2, 0.3, 0.4):
    command = step(controller, now, 99.0, lift_authorized=False)
    assert command == PumpCommand.hydraulic_hold()
    assert controller.state is not ControllerState.HOLD
assert step(controller, 0.5, 99.0, lift_authorized=False) == PumpCommand.hydraulic_hold()
assert controller.state is ControllerState.HOLD
```

增加：

```python
def test_hydraulic_hold_fails_to_all_zero_on_sensor_fault() -> None:
    controller = HeightController(control_config(), calibration())
    controller.set_target(100.0)
    assert step(controller, 0.0, 99.0, lift_authorized=False) == PumpCommand.hydraulic_hold()

    command = controller.step(
        now=0.1,
        sample=HeightSample(0.1, 100, 99.0, False, "sensor error"),
        feedback=feedback(0.1),
        lift_authorized=False,
        lower_authorized=False,
    )

    assert command == PumpCommand.safe_stop()
    assert controller.state is ControllerState.FAULT
```

在 `test_overshoot_never_auto_lowers_and_faults_only_beyond_limit` 补充：

```python
assert command.interlock is False
```

- [ ] **步骤2：运行测试并确认RED**

运行：

```powershell
py -3.13 -m pytest tests/test_controller.py::test_target_requires_500ms_continuous_tolerance_before_hold tests/test_controller.py::test_hydraulic_hold_fails_to_all_zero_on_sensor_fault tests/test_controller.py::test_overshoot_never_auto_lowers_and_faults_only_beyond_limit -q
```

预期：稳定窗口仍返回 `safe_stop()`，保持命令断言失败；故障和超调全零断言继续通过。

- [ ] **步骤3：只在目标稳定窗口返回液压保持**

在 `_automatic_command()` 的 `abs(error) <= tolerance_mm` 分支中保留稳定计时与状态转换，只把返回值改为：

```python
return PumpCommand.hydraulic_hold()
```

超调、故障、无目标、未授权非零PWM和退出路径不得改用 `hydraulic_hold()`。

- [ ] **步骤4：运行控制器定向测试并确认GREEN**

运行：

```powershell
py -3.13 -m pytest tests/test_controller.py -q
```

预期：全部通过，并证明松开 `u` 不会取消零PWM液压保持。

- [ ] **步骤5：提交保持控制改动**

```powershell
git add -- tests/test_controller.py src/agv_lift_height_control/controller.py
git commit -m "feat: hold hydraulic interlock in target band / 目标稳定区保持液压互锁"
```

### 任务4：在SSH TUI显示互锁字节语义

**文件：**

- 修改：`tests/test_operator_runtime.py`
- 修改：`src/agv_lift_height_control/operator_runtime.py`

- [ ] **步骤1：先写TUI失败测试**

在 `tests/test_operator_runtime.py` 增加：

```python
def test_tui_render_shows_actual_and_desired_interlock_state() -> None:
    output = io.StringIO()
    terminal = PosixAnsiTerminal(stdout=output)

    terminal.render(
        RuntimeSnapshot(
            mode="move",
            command=PumpCommand.hydraulic_hold(),
            desired_command=PumpCommand.safe_stop(),
        )
    )

    rendered = output.getvalue()
    assert "实际输出: 互锁=开 PWM=0" in rendered
    assert "期望输出: 互锁=关 PWM=0" in rendered
```

- [ ] **步骤2：运行测试并确认RED**

运行：

```powershell
py -3.13 -m pytest tests/test_operator_runtime.py::test_tui_render_shows_actual_and_desired_interlock_state -q
```

预期：失败，因为当前两行都不显示互锁。

- [ ] **步骤3：实现最小TUI显示**

在 `PosixAnsiTerminal.render()` 中把两行改为：

```python
f"实际输出: 互锁={'开' if snapshot.command.interlock else '关'} "
f"PWM={snapshot.command.lift_pwm} 阀值=0x{snapshot.command.lower_valve:02X} "
f"加速={snapshot.command.accel} 减速={snapshot.command.decel}",
f"期望输出: 互锁={'开' if snapshot.desired_command.interlock else '关'} "
f"PWM={snapshot.desired_command.lift_pwm} "
f"阀值=0x{snapshot.desired_command.lower_valve:02X} "
f"归零请求={'是' if snapshot.zero_requested else '否'}",
```

- [ ] **步骤4：运行TUI测试并确认GREEN**

运行：

```powershell
py -3.13 -m pytest tests/test_operator_runtime.py -q
```

预期：全部通过。

- [ ] **步骤5：提交TUI改动**

```powershell
git add -- tests/test_operator_runtime.py src/agv_lift_height_control/operator_runtime.py
git commit -m "feat: show pump interlock in SSH TUI / SSH界面显示泵互锁状态"
```

### 任务5：同步现场文档与维护地图

**文件：**

- 修改：`README.md`
- 修改：`docs/维护地图.md`

- [ ] **步骤1：更新README现场说明**

在首次闭环测试段增加以下事实：

```markdown
从连续起升进入末端区时，控制器先发送全零并完成一次停泵观察，再从静止状态发出短脉冲。进入目标±2 mm稳定窗口后，正常保持帧为 `217#0100000000000000`；故障、超时、退出和命令过期仍为 `217#0000000000000000`。TUI的“互锁=开/关”对应Byte0。
```

- [ ] **步骤2：更新维护地图证据链**

在 `docs/维护地图.md` 增加或更新以下映射：

- `pulse_on_s`：秒，来源为响应延迟并限制 `0.1..0.3`，生效于 `_terminal_pulse_command()`，测试为完整末端相位测试。
- `pulse_wait_s`：秒，来源为响应延迟加稳定时间并限制 `0.3..1.0`，生效于首次 `SETTLE` 和后续 `WAIT`。
- `_terminal_pulse_phase`：`None/SETTLE/ON/WAIT`，说明授权中断重置和硬件输出。
- `PumpCommand.interlock`：布尔值，保持为 `True`、安全停机为 `False`，编码到 `0x217 Byte0`。
- `HeightController._automatic_command()` 与 `_terminal_pulse_command()`：调用者、相位、副作用、失败全零和对应测试。
- `PosixAnsiTerminal.render()`：补充实际/期望互锁显示测试。
- 状态表：`terminal_pulse` 首次进入先全零；目标窗口计时期间发送保持帧；`hold` 持续保持帧。

- [ ] **步骤3：核对文档不存在旧语义**

运行：

```powershell
Select-String -Path README.md,docs\维护地图.md -Pattern 'terminal_pulse|末端|互锁|217#01|217#00'
git diff --check
```

预期：所有“进入末端即通泵”旧描述已消失；保持帧和安全全零边界均可检索，`git diff --check` 无输出。

- [ ] **步骤4：提交文档改动**

```powershell
git add -- README.md docs/维护地图.md
git commit -m "docs: explain terminal settling and hydraulic hold / 说明末端停泵与液压保持"
```

### 任务6：全量验证、审查和发布准备

**文件：**

- 验证：全部源码、测试和文档差异

- [ ] **步骤1：运行风险定向测试**

```powershell
py -3.13 -m pytest tests/test_types.py tests/test_can_pump.py tests/test_controller.py tests/test_operator_runtime.py tests/test_foreground_loop.py tests/test_simulation.py -q
```

预期：全部通过，特别是安全全零、命令过期、退出补零和液压仿真没有回归。

- [ ] **步骤2：运行全量测试与编译**

```powershell
py -3.13 -m pytest -q
py -3.13 -m compileall -q src tests
```

预期：pytest零失败，`compileall` 无输出且退出码为0。

- [ ] **步骤3：审查完整差异**

```powershell
git diff --check main...HEAD
git diff --stat main...HEAD
git status --short --branch
```

逐项确认：

- `safe_stop()`、CanPump命令过期、启动NMT窗和退出补零仍为8字节全零；
- 只有目标稳定窗口使用 `hydraulic_hold()`；
- 首次末端输出为全零，后续每次脉冲后都有完整等待；
- TUI和维护地图与实际字段、函数及测试名称一致；
- 工作区没有用户无关改动。

- [ ] **步骤4：现场发布门禁**

合并并推送后，Orange Pi 只执行一次 `80 mm` 低高度测试。用独立 `candump` 验证末端全零观察、目标窗口 `217#0100000000000000`、松开 `u` 后10秒保持以及按 `q` 后全零停止发送。任何一项不符都停止后续闭环测试并保存最新CSV。
