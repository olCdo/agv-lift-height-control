# 有限行程起升标定 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把首版起升标定收敛为 `40% PWM × 100 ms × 3 次`，在约 200 mm 的现场行程内生成可用于闭环定高的标定结果，同时消除 SSH 终端阻塞导致的控制循环超时。

**Architecture:** 保留 `HeightSource → ForegroundRuntime → LiftCalibrationSession/HeightController → CanPump` 的现有边界。起升标定会话只负责三次固定脉冲和安全观察；分析器负责判定三次是否都有效并生成 `min_stable_pwm=coarse_pwm=40` 的结果；草稿存储升级独立 schema，拒绝旧 27 次草稿；前台运行时将控制周期与 TUI 刷新周期解耦，POSIX 终端写入改为有界非阻塞。

**Tech Stack:** Python 3.10+、pytest、pymodbus 3.x、python-can 4.x、POSIX `termios`/`os.set_blocking`。

---

实施依据：[有限行程起升标定设计](../specs/2026-08-07-limited-travel-lift-calibration-design.md)。本计划不改变高度换算，仍使用 `height_mm = raw × 200 / 4096`，也不加入 96.5 mm 离地绝对偏置。

## Task 1：把起升标定状态机改成三次固定短脉冲

**Files:**

- Modify: `src/agv_lift_height_control/calibration.py:22-149`
- Modify: `src/agv_lift_height_control/calibration.py:237-397`
- Test: `tests/test_calibration.py:31-150`

- [ ] **Step 1：先写固定计划和结果分析的失败测试**

用三次 40% 试验替换测试里的 27 次辅助数据，并明确“通电位移”与“停泵上滑”是两个不同量：

```python
def complete_lift_trials() -> tuple[LiftTrial, ...]:
    return tuple(
        LiftTrial(
            pwm=40,
            repeat=repeat,
            response_delay_s=0.06 + repeat * 0.005,
            displacement_mm=4.0 + repeat,
            speed_mm_s=40.0 + repeat * 10.0,
            coast_mm=1.0 + repeat * 0.5,
            peak_current_raw=900 + repeat * 10,
            direction=1,
            success=True,
        )
        for repeat in range(1, 4)
    )


def test_lift_analysis_accepts_exactly_three_40_percent_trials() -> None:
    result = analyze_lift_trials(complete_lift_trials())

    assert result.min_stable_pwm == 40
    assert result.coarse_pwm == 40
    assert result.response_delay_s == pytest.approx(0.075)
    assert result.max_coast_mm == pytest.approx(2.5)
    assert result.peak_current_by_pwm == {40: 930}


@pytest.mark.parametrize(
    "trials",
    [
        complete_lift_trials()[:-1],
        complete_lift_trials()
        + (dataclasses.replace(complete_lift_trials()[-1], repeat=4),),
        tuple(
            dataclasses.replace(trial, pwm=45) if trial.repeat == 2 else trial
            for trial in complete_lift_trials()
        ),
    ],
)
def test_lift_analysis_rejects_noncanonical_trial_plan(
    trials: tuple[LiftTrial, ...],
) -> None:
    with pytest.raises(ValueError, match="40%.*3"):
        analyze_lift_trials(trials)
```

- [ ] **Step 2：运行定向测试并确认 RED**

Run:

```bash
python -m pytest -q tests/test_calibration.py::test_lift_analysis_accepts_exactly_three_40_percent_trials tests/test_calibration.py::test_lift_analysis_rejects_noncanonical_trial_plan
```

Expected: FAIL；现有分析器仍要求 `40..80% × 3 = 27` 次，并计算 `coarse=min+20`。

- [ ] **Step 3：增加固定标定常量并改写分析器**

在 `calibration.py` 定义可单独审查的标定常量；保留 `LIFT_PWM_LEVELS` 供旧最终标定包和控制器兼容使用：

```python
LIFT_PWM_LEVELS = tuple(range(40, 81, 5))
LIFT_CALIBRATION_PWM = 40
LIFT_CALIBRATION_REPEATS = 3
LIFT_PULSE_S = 0.100
LIFT_SETTLE_S = 0.700
LIFT_TRIAL_S = LIFT_PULSE_S + LIFT_SETTLE_S
LIFT_CALIBRATION_PLAN = tuple(
    (LIFT_CALIBRATION_PWM, repeat)
    for repeat in range(1, LIFT_CALIBRATION_REPEATS + 1)
)
```

`analyze_lift_trials()` 必须：

```python
def analyze_lift_trials(trials: tuple[LiftTrial, ...]) -> LiftCalibrationResult:
    actual = tuple((trial.pwm, trial.repeat) for trial in trials)
    if actual != LIFT_CALIBRATION_PLAN:
        raise ValueError("起升标定必须严格包含 40% PWM 的 3 次试验")
    if any(
        not trial.success
        or trial.direction != 1
        or trial.displacement_mm < 1.0
        for trial in trials
    ):
        raise ValueError("40% PWM 的三次起升必须都同向且通电位移至少 1 mm")
    return LiftCalibrationResult(
        min_stable_pwm=LIFT_CALIBRATION_PWM,
        coarse_pwm=LIFT_CALIBRATION_PWM,
        response_delay_s=max(trial.response_delay_s for trial in trials),
        max_coast_mm=max(max(0.0, trial.coast_mm) for trial in trials),
        peak_current_by_pwm={
            LIFT_CALIBRATION_PWM: max(
                trial.peak_current_raw for trial in trials
            )
        },
        trials=trials,
    )
```

- [ ] **Step 4：写会话时序、授权边界和上滑峰值的失败测试**

覆盖以下现场关键行为：

```python
def test_lift_session_runs_three_100ms_pulses_with_700ms_settle() -> None:
    session = LiftCalibrationSession(absolute_max_height_mm=200.0)
    now = 0.0
    height = 0.0

    for repeat in range(1, 4):
        command = session.step(now, sample(height, now), feedback(now), True)
        assert command.lift_pwm == 40

        now += 0.100
        height += 4.0
        command = session.step(now, sample(height, now), feedback(now), True)
        assert command == PumpCommand.safe_stop()

        # 观察期不再要求持续按 u，且记录观察期内的最大高度。
        now += 0.300
        command = session.step(now, sample(height + 2.0, now), feedback(now), False)
        assert command == PumpCommand.safe_stop()

        now += 0.400
        command = session.step(now, sample(height + 1.0, now), feedback(now), False)
        assert command == PumpCommand.safe_stop()
        if repeat < 3:
            assert not session.done
            # 下一脉冲必须等新的 u 授权，旧授权不串到下一次。
            command = session.step(now + 0.001, sample(height + 1.0, now + 0.001), feedback(now + 0.001), True)
            assert command.lift_pwm == 40
            now += 0.001
            height += 1.0

    assert session.done
    assert len(session.trials) == 3
    assert all(trial.displacement_mm >= 4.0 for trial in session.trials)
    assert all(trial.coast_mm == pytest.approx(2.0) for trial in session.trials)


def test_lift_session_losing_authorization_during_power_discards_trial() -> None:
    session = LiftCalibrationSession(absolute_max_height_mm=200.0)
    session.step(0.0, sample(0.0, 0.0), feedback(0.0), True)

    command = session.step(0.050, sample(1.0, 0.050), feedback(0.050), False)

    assert command == PumpCommand.safe_stop()
    assert session.trials == ()
    assert not session.done
```

同时保留并调整以下已有安全测试：传感器/CAN 超时、反向运动、临时上限、速度异常、有符号泵电流。反向运动只在 100 ms 通电段判定；观察期的小幅回落只作为数据，不把已完成脉冲作废。

- [ ] **Step 5：运行会话测试并确认 RED**

Run:

```bash
python -m pytest -q tests/test_calibration.py -k "lift_session or lift_analysis"
```

Expected: FAIL；现有会话仍用 300 ms 通电、700 ms 观察、27 次索引，并在观察期授权失效时清空当前试验。

- [ ] **Step 6：实现三次会话状态机**

实现要点：

1. `_current_pwm()` 永远返回 `LIFT_CALIBRATION_PWM`；`done` 在三条完整记录后为真。
2. `_begin()` 保存起点、通电段峰值电流和首次位移响应时刻。
3. `elapsed < LIFT_PULSE_S` 时才要求 `lift_authorized=True`、才允许 PWM=40、才检查运动方向、才累计通电峰值电流。
4. 首次进入 `elapsed >= LIFT_PULSE_S` 时保存 `_stop_height_mm`，之后始终返回全零。
5. 观察段持续更新 `_max_settle_height_mm`；到 `LIFT_TRIAL_S` 后完成记录。
6. `displacement_mm = stop_height - start_height`；`coast_mm = max(0, max_settle_height - stop_height)`；`speed_mm_s = displacement/LIFT_PULSE_S`。
7. 完成一条后不自动起下一脉冲；只有当下一周期收到新的 `u` 授权才 `_begin()`。
8. 三条完成后调用 `analyze_lift_trials()`；任一条不足 1 mm 或方向错误时抛出标定失败，应用层保持不保存草稿。

伪代码骨架：

```python
if self._active_start_s is None:
    return self._begin(...) if lift_authorized else safe_stop

elapsed = now - self._active_start_s
if self._stop_height_mm is None:
    if not lift_authorized:
        self._reset_active()
        return safe_stop
    self._observe_powered(...)
    if elapsed < LIFT_PULSE_S:
        return lift_command(40)
    self._stop_height_mm = sample.height_mm
    self._max_settle_height_mm = sample.height_mm
    return safe_stop

self._max_settle_height_mm = max(
    self._max_settle_height_mm,
    sample.height_mm,
)
if elapsed < LIFT_TRIAL_S:
    return safe_stop
self._finish(...)
return safe_stop
```

- [ ] **Step 7：运行 Task 1 定向测试，然后直接继续 Task 2**

Run:

```bash
python -m pytest -q tests/test_calibration.py -k "lift_session or lift_analysis"
python -m compileall -q src tests
git diff --check
```

Expected: 定向测试 PASS。此时先不提交，因为草稿存储和最终标定包还不接受单档峰值；立即继续 Task 2，避免产生全量测试不绿、运行时也不能落草稿的中间提交。

## Task 2：让标定结果、控制器和草稿存储接受单一 40% 档位

**Files:**

- Modify: `src/agv_lift_height_control/calibration.py:533-668`
- Modify: `src/agv_lift_height_control/runtime_storage.py:1-143`
- Modify: `src/agv_lift_height_control/controller.py:638-700`（仅在测试证明需要时做最小修改）
- Modify: `tests/test_calibration.py:400-540`
- Modify: `tests/test_operator_runtime.py:270-370`
- Modify: `tests/test_controller.py`

- [ ] **Step 1：写结果兼容性和草稿版本隔离的失败测试**

新增测试要求：

```python
def test_calibration_bundle_accepts_single_level_lift_result() -> None:
    bundle = CalibrationBundle(
        min_stable_pwm=40,
        coarse_pwm=40,
        response_delay_s=0.075,
        max_coast_mm=2.5,
        peak_current_by_pwm={40: 930},
        lower_min_start_valve=0x20,
        lower_comfortable_valve=0x40,
    )
    assert CalibrationBundle.from_json(bundle.to_json()) == bundle


def test_calibration_bundle_still_reads_legacy_complete_peak_map() -> None:
    bundle = legacy_bundle_with_all_pwm_peaks()
    assert CalibrationBundle.from_json(bundle.to_json()) == bundle


def test_lift_draft_v2_round_trip_uses_three_trials_and_one_peak() -> None:
    store = LiftCalibrationDraftStore(state_dir)
    store.save(analyze_lift_trials(complete_lift_trials()))

    raw = json.loads(store.path.read_text(encoding="utf-8"))
    assert raw["schema_version"] == 2
    assert len(raw["result"]["trials"]) == 3
    assert raw["result"]["peak_current_by_pwm"] == {"40": 930}
    assert store.load() == analyze_lift_trials(complete_lift_trials())


def test_lift_draft_rejects_old_v1_without_reinterpreting_it() -> None:
    store = LiftCalibrationDraftStore(state_dir)
    store.path.write_text(json.dumps(old_v1_27_trial_payload()), encoding="utf-8")

    with pytest.raises(ValueError, match="schema_version"):
        store.load()
```

补充控制器回归测试，分别让误差落在粗升区、P 区和末端脉冲区，断言任何非零起升命令都等于 40：

```python
@pytest.mark.parametrize("height_mm", [0.0, 55.0, 97.0])
def test_single_level_calibration_never_commands_above_40(
    height_mm: float,
) -> None:
    controller = controller_with_single_level_calibration(target_mm=100.0)
    command = controller.step(...sample(height_mm)...)
    assert command.lift_pwm in (0, 40)
```

- [ ] **Step 2：运行定向测试并确认 RED**

Run:

```bash
python -m pytest -q tests/test_calibration.py tests/test_operator_runtime.py tests/test_controller.py -k "single_level or draft_v2 or old_v1 or three_trials"
```

Expected: FAIL；当前 `CalibrationBundle` 和 JSON 解析要求 40..80 全档峰值，起升草稿仍是 schema v1。

- [ ] **Step 3：放宽最终标定包，但保持严格合法档位**

`CalibrationBundle.__post_init__()` 的规则改为：

```python
if self.min_stable_pwm not in LIFT_PWM_LEVELS:
    raise ValueError(...)
if self.coarse_pwm not in LIFT_PWM_LEVELS:
    raise ValueError(...)
if self.coarse_pwm < self.min_stable_pwm:
    raise ValueError(...)

peak_levels = set(self.peak_current_by_pwm)
required_levels = {
    level
    for level in LIFT_PWM_LEVELS
    if self.min_stable_pwm <= level <= self.coarse_pwm
}
if not required_levels.issubset(peak_levels):
    raise ValueError(...)
if not peak_levels.issubset(set(LIFT_PWM_LEVELS)):
    raise ValueError(...)
```

这样新结果 `{40: peak}` 合法，原有全档映射也继续可读。JSON 键仍必须是规范十进制字符串，继续拒绝 `"040"`、未知键、布尔值和负电流。

最终标定包的 `CALIBRATION_SCHEMA_VERSION` 保持不变，因为只扩大了同一字段的合法子集且需要继续读取已确认的旧最终标定包；起升动作草稿另行升版。

- [ ] **Step 4：把起升草稿升级为 schema v2**

在 `runtime_storage.py`：

```python
DRAFT_SCHEMA_VERSION = 2
```

加载时仍要求精确顶层键；峰值键先验证规范字符串，再转 `int`。v2 只允许 `analyze_lift_trials()` 重新分析后得到完全相同的三次结果：

```python
canonical = analyze_lift_trials(result.trials)
if canonical != result:
    raise ValueError("起升标定草稿摘要与三次原始试验不一致")
```

不迁移 v1，不把旧 27 次记录截断，不静默猜测 40% 结果。现场必须重新执行三次短脉冲生成新草稿。

- [ ] **Step 5：验证控制器在单档结果下自然退化为固定 40%**

现有控制器的粗升使用 `coarse_pwm`，P 区在 `[min_stable_pwm, coarse_pwm]` 内量化，末端使用 `min_stable_pwm`。当两者都为 40 时理论上无需修改生产代码。只有定向测试证明某一路径能产生 45..80 时，才在 `_automatic_command()` 做最小修复，禁止添加第二套 PWM 常量。

电流保护按实际发送的 40% 从 `{40: peak}` 取阈值，缺少当前档峰值时必须故障，不能回退到虚构峰值。

- [ ] **Step 6：运行 Task 2 测试并提交**

Run:

```bash
python -m pytest -q tests/test_calibration.py tests/test_operator_runtime.py tests/test_controller.py
python -m compileall -q src tests
git diff --check
```

Expected: PASS；旧最终标定包兼容测试通过，旧起升草稿拒绝测试通过。

Task 1 与 Task 2 作为同一个原子提交：

```bash
git add src/agv_lift_height_control/calibration.py src/agv_lift_height_control/runtime_storage.py src/agv_lift_height_control/controller.py tests/test_calibration.py tests/test_operator_runtime.py tests/test_controller.py
git commit -m "feat: calibrate lift with three short pulses / 使用三次短脉冲标定起升"
```

若 `controller.py` 没有实际差异，从 `git add` 参数中删除它。

## Task 3：把 50 Hz 控制循环与约 5 Hz 非阻塞 TUI 解耦

**Files:**

- Modify: `src/agv_lift_height_control/application.py:251-482`
- Modify: `src/agv_lift_height_control/operator_runtime.py:80-160`
- Modify: `tests/test_foreground_loop.py`
- Modify: `tests/test_operator_runtime.py:130-210`

- [ ] **Step 1：写 TUI 刷新节流的失败测试**

使用现有 fake clock/terminal，在 1 秒内执行约 50 个控制周期，断言控制源仍每周期调用但渲染不超过 6 次：

```python
def test_foreground_runtime_controls_at_50hz_but_renders_about_5hz() -> None:
    terminal = FakeTerminal()
    source = CountingCommandSource(stop_after_steps=51)
    runtime = make_runtime(terminal=terminal, loop_period_s=0.020)

    runtime.run(source)

    assert source.step_calls == 51
    assert 5 <= terminal.render_calls <= 6
```

再覆盖渲染异常仍走已有故障归零路径，不能因节流吞掉真正的编程错误。

- [ ] **Step 2：运行刷新测试并确认 RED**

Run:

```bash
python -m pytest -q tests/test_foreground_loop.py -k "renders_about_5hz"
```

Expected: FAIL；当前每个 20 ms 周期都调用 `terminal.render()`，约 50 Hz。

- [ ] **Step 3：在 ForegroundRuntime 中增加显示周期**

构造参数新增：

```python
render_period_s: float = 0.200
```

运行开始时初始化下一次刷新时刻；每个控制周期仍完成输入读取、安全门禁、命令更新和 CSV 记录，只节流 TUI：

```python
if now >= next_render_at:
    self.terminal.render(self._last_snapshot)
    next_render_at = now + self.render_period_s
```

不得降低以下频率：

- 控制循环：20 ms；
- CAN 完整 `0x217` 发送：50 ms，由独立泵线程保持；
- 传感器/CAN 新鲜度和 100 ms deadline 检查：每个控制周期；
- CSV cycle 记录：保持现状。

- [ ] **Step 4：写 POSIX 非阻塞输出的失败测试**

将 OS 操作注入或 monkeypatch，覆盖：

```python
def test_posix_terminal_drops_blocked_frame_without_blocking_control() -> None:
    terminal = PosixAnsiTerminal(stdin=fake_stdin, stdout=fake_stdout)
    terminal.open()
    os_write.side_effect = BlockingIOError()

    terminal.render(snapshot())

    assert terminal.dropped_frames == 1


def test_posix_terminal_drops_partial_frame_and_next_render_is_complete() -> None:
    os_write.side_effect = [5, full_frame_length]
    terminal.render(first_snapshot())
    terminal.render(second_snapshot())

    assert second_os_write_payload.startswith(b"\x1b[H")
    assert second_os_write_payload.endswith(b"\x1b[J")


def test_posix_terminal_restores_stdout_blocking_and_termios_on_close() -> None:
    terminal.open()
    terminal.close()

    os_set_blocking.assert_has_calls([call(fd, False), call(fd, True)])
    termios.tcsetattr.assert_called()
```

普通 `StringIO` 的既有渲染测试继续使用同步 fallback，确保 Windows 单元测试和文本内容断言不受影响。

- [ ] **Step 5：实现有界非阻塞帧写入**

`PosixAnsiTerminal.open()` 在保存原 termios 后：

1. 保存 stdout fd 和原 `os.get_blocking(fd)`；
2. `os.set_blocking(fd, False)`；
3. 进入 cbreak、隐藏光标；
4. 任一步失败都按相反顺序恢复已修改的状态。

`render()` 每次先在内存中构造完整 ANSI 帧，再只调用一次 `os.write()`：

```python
payload = ("\x1b[H" + "\n".join(lines) + "\x1b[J").encode()
try:
    written = os.write(self._stdout_fd, payload)
except BlockingIOError:
    self.dropped_frames += 1
    return
if written != len(payload):
    self.dropped_frames += 1
```

部分写入的剩余字节不循环重试，下一次刷新直接从 `ESC[H` 发送新完整帧。这样 SSH 背压不会占住控制线程。

`close()` 在 stdout 仍为非阻塞时只尝试一次显示光标，然后恢复 termios 和 stdout 原 blocking 状态；所有恢复步骤都应 best-effort，但不能掩盖主运行错误，也不能为了显示光标再次阻塞控制程序退出。

- [ ] **Step 6：运行 Task 3 测试并提交**

Run:

```bash
python -m pytest -q tests/test_foreground_loop.py tests/test_operator_runtime.py
python -m compileall -q src tests
git diff --check
```

Expected: PASS；50 Hz 控制计数不变，TUI 约 5 Hz，阻塞/部分写入均在一次系统调用内返回。

Commit:

```bash
git add src/agv_lift_height_control/application.py src/agv_lift_height_control/operator_runtime.py tests/test_foreground_loop.py tests/test_operator_runtime.py
git commit -m "fix: decouple nonblocking TUI from control loop / 解耦非阻塞终端与控制循环"
```

## Task 4：更新现场文档、追踪地图并完成全量验证

**Files:**

- Modify: `README.md:139-204`
- Modify: `docs/维护地图.md:35-121`
- Modify: `tests/test_repository_files.py`

- [ ] **Step 1：先写文档事实门禁测试**

测试应锁定新的现场事实，避免以后又退回 27 次：

```python
def test_readme_describes_three_40_percent_100ms_lift_trials() -> None:
    readme = Path("README.md").read_text(encoding="utf-8")
    assert "40%" in readme
    assert "100 ms" in readme
    assert "3 次" in readme
    assert "40–80%" not in readme


def test_maintenance_map_links_new_timing_to_tests() -> None:
    maintenance = Path("docs/维护地图.md").read_text(encoding="utf-8")
    assert "LIFT_PULSE_S" in maintenance
    assert "0.100 s" in maintenance
    assert "test_lift_session_runs_three_100ms_pulses_with_700ms_settle" in maintenance
```

- [ ] **Step 2：运行文档测试并确认 RED**

Run:

```bash
python -m pytest -q tests/test_repository_files.py
```

Expected: FAIL；README/维护地图仍描述 40..80%、300 ms、27 次。

- [ ] **Step 3：更新 README 现场门禁**

README 明确写明：

- 只有 40% PWM；每次通电 100 ms、全零观察 700 ms；总计三次；
- 通电时必须持续按 `u`，观察期可松开；下一次必须重新按 `u`；
- 完整三次预计动作位移远小于原 27 次，仍必须在设备旁并准备物理断电；
- CSV 即使中断也保留，`lift-calibration-draft.json` 只有三次全部成功才写；
- 旧 v1 起升草稿不会复用，需要重新标定；
- 40% 是首版闭环的唯一自动起升档位，下降仍只允许人工死手；
- 高度仍为相对起升行程，不是平台离地绝对高度。

- [ ] **Step 4：更新维护地图的变量→函数→测试链**

至少更新以下条目：

| 变量 | 单位/范围 | 生效函数 | 修改影响 | 对应测试 |
|---|---|---|---|---|
| `LIFT_CALIBRATION_PWM` | `%`，固定 40 | `LiftCalibrationSession.step()`、`analyze_lift_trials()` | 标定动作与首版闭环上限 | 三次脉冲、单档控制器测试 |
| `LIFT_PULSE_S` | `s`，0.100 | `LiftCalibrationSession.step()`、`_finish()` | 通电位移、速度、单次动作风险 | 100 ms 时序测试 |
| `LIFT_SETTLE_S` | `s`，0.700 | `LiftCalibrationSession.step()` | 上滑峰值和下一次授权时机 | 700 ms 观察测试 |
| `render_period_s` | `s`，0.200 | `ForegroundRuntime.run()` | SSH 显示频率，不影响控制/CAN | 50 Hz 控制/5 Hz 渲染测试 |

函数地图里把 `PosixAnsiTerminal.render()` 的副作用改为“单次非阻塞完整帧写入，阻塞或部分帧丢弃”；把 `LiftCalibrationSession.step()` 改为新的三阶段授权逻辑。

- [ ] **Step 5：执行全量本地验证**

Run:

```bash
python -m pytest -q
python -m compileall -q src tests
git diff --check
git status --short
```

Expected:

- pytest 全绿；
- compileall 无输出且退出码 0；
- `git diff --check` 无输出；
- status 只包含本任务预期文档变更。

- [ ] **Step 6：提交文档与门禁测试**

Commit:

```bash
git add README.md docs/维护地图.md tests/test_repository_files.py
git commit -m "docs: update limited-travel field procedure / 更新有限行程现场流程"
```

## Task 5：Orange Pi 现场验证，不在开发机代替硬件结论

本任务不由自动测试声称完成；必须由设备旁操作者在 Orange Pi 上执行。每一步异常都先停，保留最新 CSV，不跳过门禁。

- [ ] **Step 1：更新、安装并回归**

```bash
cd ~/agv-lift-height-control
git pull --ff-only
source ~/.venv/bin/activate
python -m pip install -e ".[dev]"
python -m pytest -q
```

- [ ] **Step 2：确认独占串口/CAN和 500 kbit/s**

```bash
fuser -v /dev/ttyS7
ps -ef | grep -Ei '[o]penplc|[k]inco_duolun|[a]gv_lift_height_control'
ip -details link show can0
```

Expected: `/dev/ttyS7` 无占用、没有其他控制程序、`can0` 为 UP/ERROR-ACTIVE 且 bitrate 500000。

- [ ] **Step 3：只读传感器和被动 CAN**

```bash
python -m agv_lift_height_control --config config/local.json monitor --duration-s 60
python -m agv_lift_height_control --config config/local.json observe-can --duration-s 60
```

Expected: 高度连续有效；`0x197` 故障码为 `0x00` 或已解释状态；无控制输出。

- [ ] **Step 4：执行三次短脉冲起升标定**

```bash
python -m agv_lift_height_control \
  --config config/local.json \
  calibrate-lift --temporary-max-mm 200
```

操作：每次只在屏幕等待授权时按住/重复按 `u`；看到 PWM 归零后可松开；等下一次提示再重新按 `u`。三次期间在设备旁观察，有异常立即物理断电或 `Ctrl+C`。

Expected:

- 只有三次 `PWM=40`；
- 每次约 100 ms 通电，之后约 700 ms 全零；
- 完成后生成 v2 起升草稿；
- 三次任一通电位移不足 1 mm则失败且不生成新草稿，此时不要擅自升到 80%，回传 CSV 再决定。

- [ ] **Step 5：检查草稿和 CSV**

```bash
python - <<'PY'
import json
from pathlib import Path

state = Path.home() / ".local/state/agv-lift-height-control"
draft = json.loads((state / "lift-calibration-draft.json").read_text())
print(json.dumps(draft, ensure_ascii=False, indent=2))
print("最新CSV:", max((state / "logs").glob("*calibrate-lift*.csv")))
PY
```

Expected: `schema_version=2`、三条试验、`min_stable_pwm=40`、`coarse_pwm=40`、峰值电流只有 `"40"`。

- [ ] **Step 6：完成下降标定和低高度闭环**

```bash
python -m agv_lift_height_control --config config/local.json calibrate-lower
python -m agv_lift_height_control --config config/local.json confirm-lower \
  --comfortable-valve 0x40
python -m agv_lift_height_control --config config/local.json show-calibration

python -m agv_lift_height_control --config config/local.json move \
  --target-mm 50 --temporary-max-mm 200
```

先在 50 mm 完成三次闭环；每次持续按 `u`，停止授权应立即全零。确认误差、超调和回落后，再逐级测试 80 mm、100 mm，不直接跳到 200 mm。

现场验收标准：

- 连续 500 ms 位于目标 ±2 mm后进入保持；
- 超调不超过 5 mm；
- 不出现自动下降；
- SSH 卡顿或断开时 CAN 独立 watchdog 归零；
- 所有 CSV 可追踪实际命令、期望命令和授权事件。

## 最终完成门禁

- [ ] 两个实现提交和一个文档提交均为英中双语 commit message；设计与计划文档提交也保持英中双语。
- [ ] `python -m pytest -q`、compileall、diff-check 全部通过。
- [ ] 独立代码审查无 Critical/Important。
- [ ] `main` 只快进合并，不重写历史。
- [ ] 推送后 Orange Pi 用 `git pull --ff-only` 更新。
- [ ] 未连接真实硬件前，不宣称 ±2 mm/超调 ≤5 mm 已现场验证。
