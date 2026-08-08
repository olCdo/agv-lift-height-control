# 下降标定前安全预升模式实施计划

> **面向代理执行者：** 必须逐项执行本计划并勾选复核。实施时使用 `subagent-driven-development`（推荐）或 `executing-plans`；所有生产代码必须先有能够按预期失败的测试。

**目标：** 新增 `prepare-lower` SSH前台命令，使用当前schema 3起升草稿中的40% PWM，以100 ms脉冲和700 ms全零观察把传感器高度预升到100 mm，同时受持续授权、200 mm临时上限及现有运行时安全门禁保护。

**架构：** 在现有标定模块增加不接触硬件的 `PrepareLowerSession` 状态机，由应用层 `PrepareLowerCommandSource` 转换为统一 `CommandDecision`。`ForegroundRuntime` 继续负责TTY授权、5秒NMT安全窗、20 ms循环、传感器/CAN新鲜度、速度与退出归零；`CanPump` 仍是唯一0x217发送者。

**技术栈：** Python 3.10+、pytest、pymodbus 3.x、python-can 4.x、现有 `HeightSample`/`PumpFeedback`/`PumpCommand` 公共类型。

---

## 文件结构与职责

- 修改 `src/agv_lift_height_control/cli.py`：声明命令及两个必填高度参数。
- 修改 `src/agv_lift_height_control/calibration.py`：定义预升常量、状态和纯状态机会话。
- 修改 `src/agv_lift_height_control/application.py`：读取起升草稿、计算有效上限、接入前台运行时并打印结果。
- 修改 `src/agv_lift_height_control/__init__.py`：导出预升会话和状态，保持公共接口可发现。
- 修改 `tests/test_cli_application.py`：验证CLI暴露和参数边界。
- 修改 `tests/test_calibration.py`：验证脉冲、观察、授权、完成、方向、上限和过流。
- 修改 `tests/test_application_modes.py`：验证草稿前置门禁、硬件构造、有效上限、完成输出及草稿不变。
- 修改 `tests/test_foreground_loop.py`：验证预升状态、目标和故障进入TUI/CSV快照。
- 修改 `README.md`：增加现场操作顺序和命令。
- 修改 `docs/维护地图.md`：补齐预升变量、函数、调用链和测试映射。

不新增硬件驱动文件，不修改Modbus比例，不修改CAN协议编解码，不新增持久化格式。

### 任务1：增加CLI命令和参数门禁

**文件：**

- 修改：`tests/test_cli_application.py`
- 修改：`src/agv_lift_height_control/cli.py`

- [ ] **步骤1：先写CLI失败测试**

在命令枚举中加入：

```python
("prepare-lower", "--target-mm", "100", "--temporary-max-mm", "200"),
```

并增加：

```python
@pytest.mark.parametrize(
    "args",
    [
        ("prepare-lower",),
        ("prepare-lower", "--target-mm", "100"),
        ("prepare-lower", "--temporary-max-mm", "200"),
        (
            "prepare-lower",
            "--target-mm",
            "nan",
            "--temporary-max-mm",
            "200",
        ),
    ],
)
def test_prepare_lower_requires_finite_target_and_temporary_limit(args) -> None:
    with pytest.raises(SystemExit):
        parse(*args)
```

- [ ] **步骤2：运行测试并确认RED**

运行：

```bash
python -m pytest tests/test_cli_application.py -q
```

预期：命令枚举用例因 `prepare-lower` 不是合法命令而失败；参数缺失用例由argparse拒绝。

- [ ] **步骤3：实现最小CLI定义**

在 `calibrate-lift` 与 `calibrate-lower` 之间加入：

```python
prepare_lower = subparsers.add_parser(
    "prepare-lower", help="使用起升草稿为下降标定安全预升"
)
prepare_lower.add_argument(
    "--target-mm",
    required=True,
    type=_bounded_float("预升目标高度", 0.001, 2900.0),
)
prepare_lower.add_argument(
    "--temporary-max-mm",
    required=True,
    type=_bounded_float("预升临时最大高度", 0.001, 2900.0),
)
```

- [ ] **步骤4：运行测试并确认GREEN**

运行：

```bash
python -m pytest tests/test_cli_application.py -q
```

预期：本文件全部通过。

- [ ] **步骤5：提交CLI改动**

```bash
git add src/agv_lift_height_control/cli.py tests/test_cli_application.py
git commit -m "feat: add prepare-lower CLI / 新增下降标定预升命令"
```

### 任务2：实现预升脉冲与观察状态机

**文件：**

- 修改：`tests/test_calibration.py`
- 修改：`src/agv_lift_height_control/calibration.py`
- 修改：`src/agv_lift_height_control/__init__.py`

- [ ] **步骤1：先写脉冲、观察、授权和完成测试**

向导入列表加入 `PREPARE_LOWER_PULSE_S`、`PREPARE_LOWER_SETTLE_S`、`PrepareLowerSession` 和 `PrepareLowerState`，并增加工厂：

```python
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
```

增加行为测试：

```python
def test_prepare_lower_uses_40_percent_100ms_pulses_and_700ms_settle() -> None:
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
    session.step(
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

    assert command == PumpCommand.safe_stop()
    assert session.done is True
    assert session.state is PrepareLowerState.DONE
    assert session.final_height_mm == pytest.approx(100.1)


def test_prepare_lower_requires_room_for_measured_coast() -> None:
    with pytest.raises(CalibrationError, match="上滑.*安全空间"):
        prepare_session(target=198.0, upper=200.0)
```

- [ ] **步骤2：运行测试并确认RED**

运行：

```bash
python -m pytest tests/test_calibration.py -q
```

预期：测试收集阶段因预升类型和常量尚不存在而失败。

- [ ] **步骤3：实现状态、构造校验和基础周期**

在 `calibration.py` 中增加 `Enum` 导入、常量和状态：

```python
from enum import Enum

PREPARE_LOWER_PULSE_S = 0.1
PREPARE_LOWER_SETTLE_S = 0.7


class PrepareLowerState(str, Enum):
    WAIT_AUTH = "待授权"
    PULSE = "预升脉冲"
    SETTLE = "停泵观察"
    DONE = "完成"
    FAULT = "故障"
```

实现 `PrepareLowerSession`。公开属性固定为 `target_mm`、`effective_max_height_mm`、`state`、`fault_reason`、`final_height_mm`，硬件输出仅从 `step()` 返回：

```python
class PrepareLowerSession:
    """用已验证最低PWM短脉冲预升，为下降标定准备传感器行程。"""

    def __init__(
        self,
        lift: LiftCalibrationResult,
        *,
        target_mm: float,
        effective_max_height_mm: float,
        direction_tolerance_mm: float,
        sensor_timeout_s: float,
        feedback_timeout_s: float,
        current_multiplier: float,
        current_duration_s: float,
    ) -> None:
        if not isinstance(lift, LiftCalibrationResult):
            raise CalibrationError("预升必须使用有效起升标定草稿")
        self.target_mm = _finite_number("预升目标高度", target_mm, minimum=0.001)
        self.effective_max_height_mm = _finite_number(
            "预升有效最大高度", effective_max_height_mm, minimum=0.001
        )
        if self.effective_max_height_mm > 2900.0:
            raise CalibrationError("预升有效最大高度不得超过2900 mm")
        if self.target_mm + lift.max_coast_mm > self.effective_max_height_mm:
            raise CalibrationError("预升目标未给最大停泵上滑保留安全空间")
        self._pwm = _strict_int("预升PWM", lift.min_stable_pwm, 1, 100)
        peak = lift.peak_current_by_pwm.get(self._pwm)
        if type(peak) is not int or peak <= 0:
            raise CalibrationError("起升草稿缺少最低稳定PWM峰值电流")
        self._overcurrent_threshold = peak * _finite_number(
            "过流倍数", current_multiplier, minimum=1.0
        )
        self._current_duration_s = _session_timeout(
            "过流持续时间", current_duration_s, 0.2
        )
        self._direction_tolerance_mm = _finite_number(
            "direction_tolerance_mm", direction_tolerance_mm, minimum=0
        )
        self._sensor_timeout_s = _session_timeout(
            "sensor_timeout_s", sensor_timeout_s, 0.1
        )
        self._feedback_timeout_s = _session_timeout(
            "feedback_timeout_s", feedback_timeout_s, 0.15
        )
        self.state = PrepareLowerState.WAIT_AUTH
        self.fault_reason: str | None = None
        self.final_height_mm: float | None = None
        self._last_now: float | None = None
        self._initial_height: float | None = None
        self._cycle_start_height: float | None = None
        self._pulse_started_at: float | None = None
        self._settle_started_at: float | None = None
        self._overcurrent_since: float | None = None
        self._started_output = False

    @property
    def done(self) -> bool:
        return self.state is PrepareLowerState.DONE

    @property
    def failed(self) -> bool:
        return self.state is PrepareLowerState.FAULT

    def step(
        self,
        *,
        now: float,
        sample: HeightSample,
        feedback: PumpFeedback | None,
        lift_authorized: bool,
    ) -> PumpCommand:
        if type(lift_authorized) is not bool:
            raise CalibrationError("lift_authorized 必须是bool")
        if self.done or self.failed:
            return PumpCommand.safe_stop()
        try:
            timestamp, height, checked_feedback = _validate_session_inputs(
                now=now,
                sample=sample,
                feedback=feedback,
                last_now=self._last_now,
                sensor_timeout_s=self._sensor_timeout_s,
                feedback_timeout_s=self._feedback_timeout_s,
                absolute_max_height_mm=self.effective_max_height_mm,
            )
            self._last_now = timestamp
            self.final_height_mm = height
            if self._initial_height is None:
                self._initial_height = height
            if self._started_output and height >= self.target_mm:
                self.state = PrepareLowerState.DONE
                return PumpCommand.safe_stop()
            if self.state is PrepareLowerState.WAIT_AUTH:
                return self._wait_or_start(timestamp, height, lift_authorized)
            if self.state is PrepareLowerState.PULSE:
                return self._advance_pulse(timestamp, lift_authorized)
            return self._advance_settle(timestamp, height, lift_authorized)
        except CalibrationError as exc:
            return self._fail(str(exc))

    def _wait_or_start(
        self, now: float, height: float, lift_authorized: bool
    ) -> PumpCommand:
        if not lift_authorized:
            return PumpCommand.safe_stop()
        self.state = PrepareLowerState.PULSE
        self._pulse_started_at = now
        self._settle_started_at = None
        self._cycle_start_height = height
        self._started_output = True
        return PumpCommand(interlock=True, lift_pwm=self._pwm)

    def _advance_pulse(self, now: float, lift_authorized: bool) -> PumpCommand:
        assert self._pulse_started_at is not None
        if not lift_authorized:
            return self._enter_settle(now)
        if now - self._pulse_started_at + 1e-12 < PREPARE_LOWER_PULSE_S:
            return PumpCommand(interlock=True, lift_pwm=self._pwm)
        return self._enter_settle(self._pulse_started_at + PREPARE_LOWER_PULSE_S)

    def _enter_settle(self, started_at: float) -> PumpCommand:
        self.state = PrepareLowerState.SETTLE
        self._pulse_started_at = None
        self._settle_started_at = started_at
        return PumpCommand.safe_stop()

    def _advance_settle(
        self, now: float, height: float, lift_authorized: bool
    ) -> PumpCommand:
        assert self._settle_started_at is not None
        if now - self._settle_started_at + 1e-12 < PREPARE_LOWER_SETTLE_S:
            return PumpCommand.safe_stop()
        self.state = PrepareLowerState.WAIT_AUTH
        self._settle_started_at = None
        self._cycle_start_height = None
        return self._wait_or_start(now, height, lift_authorized)

    def _fail(self, reason: str) -> PumpCommand:
        self.state = PrepareLowerState.FAULT
        self.fault_reason = reason
        self._pulse_started_at = None
        self._settle_started_at = None
        return PumpCommand.safe_stop()
```

基础周期必须保持以下语义：开始脉冲时记录周期起点；通电达到100 ms后进入700 ms观察；通电期间撤权也进入完整观察，防止快速重新授权绕过脉冲上限；观察结束且仍授权时可以开始下一次完整脉冲。首次样本、方向和电流门禁在下一任务以独立RED测试加入。

- [ ] **步骤4：导出新类型并运行GREEN测试**

在 `__init__.py` 的标定导入和 `__all__` 中加入：

```python
PREPARE_LOWER_PULSE_S,
PREPARE_LOWER_SETTLE_S,
PrepareLowerSession,
PrepareLowerState,
```

运行：

```bash
python -m pytest tests/test_calibration.py -q
```

预期：基础状态机测试及原有标定测试全部通过。

- [ ] **步骤5：提交基础状态机**

```bash
git add src/agv_lift_height_control/calibration.py src/agv_lift_height_control/__init__.py tests/test_calibration.py
git commit -m "feat: add pre-lift pulse session / 新增安全预升脉冲会话"
```

### 任务3：补齐预升专用安全门禁

**文件：**

- 修改：`tests/test_calibration.py`
- 修改：`src/agv_lift_height_control/calibration.py`

- [ ] **步骤1：先写方向、上限、起点和持续过流测试**

```python
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
    assert "有效最大高度" in (session.fault_reason or "")
```

测试工厂的起升峰值为930 raw，1.5倍阈值为1395 raw，因此1400能够明确越过门限，并验证负极性也按绝对值处理。

- [ ] **步骤2：运行测试并确认RED**

运行：

```bash
python -m pytest tests/test_calibration.py -q
```

预期：新增门禁用例因辅助方法尚未完整实现而失败。

- [ ] **步骤3：实现门禁辅助方法和失败锁存**

实现以下逻辑：

```python
def _initialize_start_height(self, height: float) -> None:
    if self._initial_height is not None:
        return
    self._initial_height = height
    if self.target_mm <= height:
        raise CalibrationError("预升目标必须高于启动高度")

def _guard_hard_limit(self, height: float) -> None:
    if height >= self.effective_max_height_mm:
        raise CalibrationError("预升高度达到有效最大高度")

def _guard_direction(self, height: float) -> None:
    if (
        self._cycle_start_height is not None
        and height < self._cycle_start_height - self._direction_tolerance_mm
    ):
        raise CalibrationError("预升期间高度方向反向")

def _guard_overcurrent(self, now: float, feedback: PumpFeedback) -> None:
    if abs(feedback.current_raw) <= self._overcurrent_threshold:
        self._overcurrent_since = None
        return
    if self._overcurrent_since is None:
        self._overcurrent_since = now
        return
    if now - self._overcurrent_since + 1e-12 >= self._current_duration_s:
        raise CalibrationError("预升泵电流持续过流")

def _fail(self, reason: str) -> PumpCommand:
    self.state = PrepareLowerState.FAULT
    self.fault_reason = reason
    self._pulse_started_at = None
    self._settle_started_at = None
    return PumpCommand.safe_stop()
```

在 `step()` 中用 `_initialize_start_height(height)` 替换任务2的简单首次高度赋值，并在判断目标完成之前依次调用：

```python
self._guard_hard_limit(height)
self._guard_direction(height)
self._guard_overcurrent(timestamp, checked_feedback)
```

`_wait_or_start()`、`_advance_pulse()` 和 `_advance_settle()` 必须只返回 `PumpCommand.safe_stop()` 或 `PumpCommand(interlock=True, lift_pwm=self._pwm)`，任何路径的 `lower_valve` 都保持零。

- [ ] **步骤4：运行标定全文件并确认GREEN**

```bash
python -m pytest tests/test_calibration.py -q
```

预期：新增门禁与全部原有标定用例通过。

- [ ] **步骤5：提交安全门禁**

```bash
git add src/agv_lift_height_control/calibration.py tests/test_calibration.py
git commit -m "fix: guard safe pre-lift motion / 增强安全预升动作门禁"
```

### 任务4：接入应用编排、草稿门禁和运行快照

**文件：**

- 修改：`tests/test_application_modes.py`
- 修改：`tests/test_foreground_loop.py`
- 修改：`src/agv_lift_height_control/application.py`

- [ ] **步骤1：先写草稿前置门禁和构造测试**

在 `test_application_modes.py` 增加：

```python
def test_prepare_lower_requires_lift_draft_before_hardware_factories(tmp_path) -> None:
    deps, calls, _pump, _observer, _lock = harness(
        tmp_path, [TerminalEvent.keypress("q")]
    )

    with pytest.raises(CalibrationError, match="草稿"):
        run_application(arguments(tmp_path, "prepare-lower"), dependencies=deps)

    assert calls == ["foreground"]


def test_prepare_lower_builds_from_draft_without_final_bundle(tmp_path) -> None:
    config_file = config_path(tmp_path)
    config = load_config(config_file)
    store = CalibrationStore(tmp_path / "state" / "calibration.json")
    lift = lift_result()

    source = _build_control_source(
        Namespace(
            command="prepare-lower",
            target_mm=100.0,
            temporary_max_mm=200.0,
        ),
        config,
        store,
        lift_draft=lift,
    )

    assert source.allow_lift is True
    assert source.allow_lower is False
    assert source.session.target_mm == 100.0
    assert source.session.effective_max_height_mm == 200.0
    assert source.session._pwm == 40


def test_prepare_lower_effective_limit_uses_persistent_soft_limit(tmp_path) -> None:
    config_file = config_path(tmp_path)
    config = load_config(config_file)
    store = CalibrationStore(tmp_path / "state" / "calibration.json")
    store.save(final_bundle(soft_limit=150.0))

    source = _build_control_source(
        Namespace(
            command="prepare-lower",
            target_mm=100.0,
            temporary_max_mm=200.0,
        ),
        config,
        store,
        lift_draft=lift_result(),
    )

    assert source.session.effective_max_height_mm == 150.0
```

- [ ] **步骤2：运行应用测试并确认RED**

```bash
python -m pytest tests/test_application_modes.py -q
```

预期：命令未加载草稿、构造函数不接受 `lift_draft` 或没有预升命令源而失败。

- [ ] **步骤3：实现命令源和前置加载**

新增：

```python
class PrepareLowerCommandSource:
    """把纯预升会话适配为前台运行时命令源。"""

    allow_lift = True
    allow_lower = False
    controller = None

    def __init__(self, session: PrepareLowerSession) -> None:
        self.session = session
        self.status = session

    def step(
        self, now, sample, feedback, lift_authorized, lower_authorized
    ) -> CommandDecision:
        if sample is None or feedback is None:
            return CommandDecision(PumpCommand.safe_stop())
        command = self.session.step(
            now=now,
            sample=sample,
            feedback=feedback,
            lift_authorized=lift_authorized,
        )
        return CommandDecision(
            command,
            done=self.session.done,
            fatal_reason=self.session.fault_reason if self.session.failed else None,
        )
```

把草稿前置加载改为：

```python
lift_draft = (
    draft_store.load_lift()
    if mode in {"calibrate-lower", "prepare-lower"}
    else None
)
```

并把构造调用改为：

```python
source = _build_control_source(
    args,
    config,
    calibration_store,
    lift_draft=lift_draft,
)
```

`_build_control_source()` 增加仅关键字参数 `lift_draft=None`。在最终标定包强制加载之前处理 `prepare-lower`：

```python
if args.command == "prepare-lower":
    if lift_draft is None:
        raise CalibrationError("预升必须先读取有效起升标定草稿")
    limits = [
        float(args.temporary_max_mm),
        config.control.absolute_max_height_mm,
        2900.0,
    ]
    if calibration_store.path.exists():
        persistent = calibration_store.load().soft_upper_limit_mm
        if persistent is not None:
            limits.append(persistent)
    return PrepareLowerCommandSource(
        PrepareLowerSession(
            lift_draft,
            target_mm=args.target_mm,
            effective_max_height_mm=min(limits),
            direction_tolerance_mm=config.control.direction_tolerance_mm,
            sensor_timeout_s=config.control.sensor_timeout_s,
            feedback_timeout_s=config.can.feedback_timeout_s,
            current_multiplier=config.control.current_multiplier,
            current_duration_s=config.control.current_duration_s,
        )
    )
```

- [ ] **步骤4：先写目标、状态、故障快照测试**

在 `test_foreground_loop.py` 使用已有运行时假对象增加：

```python
def test_prepare_lower_status_source_populates_target_state_and_fault_snapshot() -> None:
    source = SimpleNamespace(
        controller=None,
        status=SimpleNamespace(
            target_mm=100.0,
            state=PrepareLowerState.SETTLE,
            fault_reason="预升测试故障",
        ),
    )
    foreground = runtime()
    foreground.mode = "prepare-lower"
    value = foreground._snapshot(
        source,
        HeightSample(1.0, 20, 20.0, True, None),
        PumpFeedback(1.0, 0, 0, 0),
        PumpCommand.safe_stop(),
        PumpCommand.safe_stop(),
    )

    assert value.target_mm == 100.0
    assert value.target_error_mm == 80.0
    assert value.controller_state == "停泵观察"
    assert value.controller_fault == "预升测试故障"
```

运行：

```bash
python -m pytest tests/test_foreground_loop.py -q
```

预期：目标、状态或故障为空而失败。

- [ ] **步骤5：让快照读取命令源状态对象并确认GREEN**

把 `_snapshot()` 的状态对象选择改为：

```python
controller = getattr(source, "controller", None)
status = controller or getattr(source, "status", None)
target = getattr(status, "target_mm", None)
state = getattr(status, "state", None)
state_text = getattr(state, "value", str(state) if state is not None else None)
```

并把故障字段改为：

```python
controller_fault=getattr(status, "fault_reason", None),
```

运行：

```bash
python -m pytest tests/test_application_modes.py tests/test_foreground_loop.py -q
```

预期：两个测试文件全部通过，原有控制器模式快照不变。

- [ ] **步骤6：实现成功输出和草稿不变门禁**

在 `_run_mode()` 的运行后分支加入：

```python
elif mode == "prepare-lower":
    if not source.session.done or source.session.final_height_mm is None:
        raise RuntimeError("下降标定预升未完成")
    print(
        f"预升完成，最终传感器高度: {source.session.final_height_mm:g} mm",
        file=deps.stdout,
    )
    log_path = getattr(logger, "path", None)
    if log_path is not None:
        print(f"CSV日志: {log_path}", file=deps.stdout)
```

先向 `test_application_modes.py` 的标定导入加入 `PrepareLowerState`，再写应用完成测试。该测试只替换前台循环为确定性完成动作；脉冲和归零行为已由会话及运行时测试覆盖：

```python
def test_prepare_lower_reports_completion_without_rewriting_lift_draft(
    tmp_path, monkeypatch
) -> None:
    deps, _calls, _pump, _observer, _lock = harness(tmp_path, [])
    state = tmp_path / "state"
    draft_path = state / "lift-calibration-draft.json"
    CalibrationDraftStore(draft_path).save_lift(lift_result())
    before = draft_path.read_bytes()
    logger = Logger()
    logger.path = tmp_path / "logs" / "prepare-lower.csv"
    deps.logger_factory = lambda _path, _mode: logger

    def complete(_runtime, source, *, duration_s=None):
        assert duration_s == 60.0
        source.session._started_output = True
        source.session.final_height_mm = 100.2
        source.session.state = PrepareLowerState.DONE

    monkeypatch.setattr(
        "agv_lift_height_control.application.ForegroundRuntime.run", complete
    )

    result = run_application(
        arguments(
            tmp_path,
            "prepare-lower",
            target_mm=100.0,
            temporary_max_mm=200.0,
        ),
        dependencies=deps,
    )

    assert result == 0
    assert "最终传感器高度: 100.2 mm" in deps.stdout.getvalue()
    assert "prepare-lower.csv" in deps.stdout.getvalue()
    assert draft_path.read_bytes() == before
    assert not (state / "lower-calibration-draft.json").exists()
```

- [ ] **步骤7：提交应用接线**

```bash
git add src/agv_lift_height_control/application.py tests/test_application_modes.py tests/test_foreground_loop.py
git commit -m "feat: wire safe prepare-lower runtime / 接入安全预升前台运行时"
```

### 任务5：更新现场文档和维护地图

**文件：**

- 修改：`README.md`
- 修改：`docs/维护地图.md`

- [ ] **步骤1：更新README命令表和现场顺序**

在模式表中加入：

```markdown
| `prepare-lower` | 是 | 是 | 读取起升草稿，以40%短脉冲预升到指定传感器高度，不写标定文件 |
```

在起升标定与下降标定之间加入完整命令：

```bash
python -m agv_lift_height_control \
  --config config/local.json \
  prepare-lower \
  --target-mm 100 \
  --temporary-max-mm 200
```

明确说明：设备旁持续按 `u`；每次100 ms通泵、700 ms全零观察；到100 mm后自动归零退出；当前4.297 mm上滑意味着最终高度可能略高于100 mm；严禁与OpenPLC或其他0x217发送者同时运行。

- [ ] **步骤2：补齐维护地图可追踪关系**

变量表加入：

- `PREPARE_LOWER_PULSE_S`：秒、固定0.100、在 `_advance_pulse()` 生效、对应脉冲测试。
- `PREPARE_LOWER_SETTLE_S`：秒、固定0.700、在 `_advance_settle()` 生效、对应观察和撤权绕过测试。
- `target_mm`：mm、CLI必填0.001..2900、在构造和 `step()` 生效、对应目标完成测试。
- `effective_max_height_mm`：mm、临时/持久/配置/2900最小值、对应有效上限测试。
- `_overcurrent_threshold`：raw、草稿峰值乘配置倍数、在 `_guard_overcurrent()` 生效、对应正负极性持续过流测试。

函数地图加入 `PrepareLowerSession.step()` 与 `PrepareLowerCommandSource.step()`，并把主启动流程中的现场命令分支补充 `prepare-lower -> schema 3草稿 -> 预升会话 -> ForegroundRuntime -> CanPump`。

- [ ] **步骤3：检查文档命令与真实CLI一致并提交**

运行：

```bash
python -m agv_lift_height_control --config config/example.json prepare-lower --help
git diff --check
```

预期：帮助中显示两个必填参数；差异检查无输出。

提交：

```bash
git add README.md docs/维护地图.md
git commit -m "docs: explain safe pre-lift workflow / 说明安全预升现场流程"
```

### 任务6：全量验证和交付提交检查

**文件：**

- 检查：本计划列出的全部源码、测试和文档

- [ ] **步骤1：运行定向测试**

```bash
python -m pytest \
  tests/test_cli_application.py \
  tests/test_calibration.py \
  tests/test_application_modes.py \
  tests/test_foreground_loop.py -q
```

预期：全部通过，无warning或线程异常。

- [ ] **步骤2：运行全量测试和编译检查**

```bash
python -m pytest -q
python -m compileall -q src tests
```

预期：全量pytest返回0；`compileall`无输出并返回0。

- [ ] **步骤3：审查变更范围和维护地图一致性**

```bash
git diff --check origin/main...HEAD
git status --short
git log --oneline origin/main..HEAD
```

预期：差异检查无输出；工作区干净；提交均同时包含英文和中文描述。逐项确认维护地图列出的变量、函数和测试名称可以在当前代码中检索到。

- [ ] **步骤4：现场部署前只给出安全命令，不远程代操作硬件**

Orange Pi更新与验证：

```bash
cd ~/agv-lift-height-control
git pull --ff-only
source ~/.venv/bin/activate
python -m pip install -e ".[dev]"
python -m pytest -q
```

先只读确认高度，再由设备旁操作者执行：

```bash
python -m agv_lift_height_control \
  --config config/local.json \
  monitor --duration-s 10

python -m agv_lift_height_control \
  --config config/local.json \
  prepare-lower \
  --target-mm 100 \
  --temporary-max-mm 200
```

现场验收：初始5秒无动作；只有持续按 `u` 才出现100 ms短脉冲；达到100 mm后自动停止；退出后0x217为全零；起升草稿文件内容不变；CSV包含目标、状态、实际/期望命令和退出原因。

## 计划自审结论

- 设计规格九个章节均有对应任务：CLI、状态机、安全门禁、草稿和上限、运行时、日志显示、测试、现场步骤及非目标边界。
- 类型和属性名称统一使用 `PrepareLowerSession`、`PrepareLowerState`、`target_mm`、`effective_max_height_mm`、`final_height_mm` 和 `status`。
- 计划未引入Modbus TCP、自动下降、100 mm保持、CAN网络配置修改或新持久化格式。
- 每项生产代码都有先失败后通过的定向测试步骤，所有提交信息均为英文描述加中文描述。
