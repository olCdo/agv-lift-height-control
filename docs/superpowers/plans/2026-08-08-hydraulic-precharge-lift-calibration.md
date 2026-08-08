# 液压预充压起升标定实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**目标：** 将起升标定改为一次40%预充压加三次40%正式测量，并使用完整800 ms周期的最终净位移和真实响应延迟生成闭环参数。

**架构：** 保持 LiftCalibrationSession 作为唯一标定状态机，在内部区分预充压和正式测量；预充压保留在CSV周期日志中，但不生成 LiftTrial。analyze_lift_trials() 只分析三条正式样本，起升草稿升级到schema v3并拒绝旧字段语义。

**技术栈：** Python 3.10、pytest、dataclass、严格JSON schema、现有SSH前台运行时和CSV日志。

---

## 文件映射

- 修改 src/agv_lift_height_control/calibration.py：预充压状态、完整周期观测和分析门禁。
- 修改 src/agv_lift_height_control/runtime_storage.py：起升草稿schema v3。
- 修改 tests/test_calibration.py：四次动作、延迟运动、重新授权、方向与分析测试。
- 修改 tests/test_operator_runtime.py：草稿v3往返和v1/v2拒绝。
- 修改 README.md、docs/维护地图.md、tests/test_repository_files.py：现场说明与追踪。
- 不修改 application.py：现有保存路径已经在 session.done 后调用 analyze_lift_trials()。
- 不修改 controller.py：现有闭环已经把脉冲限制为100–300 ms。

### 任务1：更新三次正式样本的分析门禁

**文件：**

- 修改：tests/test_calibration.py
- 修改：src/agv_lift_height_control/calibration.py

- [ ] **步骤1：写延迟响应的失败测试**

将测试样本的 start_delay_s 设为0.12、0.14、0.16秒，并新增：

```python
def test_lift_analysis_accepts_delayed_settled_displacement() -> None:
    result = analyze_lift_trials(complete_lift_trials())

    assert result.min_stable_pwm == result.coarse_pwm == 40
    assert result.response_delay_s == pytest.approx(0.16)
    assert result.max_coast_mm == pytest.approx(2.5)
    assert result.peak_current_by_pwm == {40: 930}


def test_lift_analysis_rejects_response_later_than_controller_limit() -> None:
    delayed = list(complete_lift_trials())
    delayed[1] = replace(delayed[1], start_delay_s=0.301, success=False)

    with pytest.raises(CalibrationError, match="响应延迟.*300 ms"):
        analyze_lift_trials(tuple(delayed))
```

把不足1 mm用例的错误断言改为匹配“观察结束净位移至少1 mm”。

- [ ] **步骤2：运行并确认RED**

```bash
python -m pytest -q tests/test_calibration.py -k "lift_analysis"
```

预期：0.301秒用例失败，旧错误文本也不匹配新语义。

- [ ] **步骤3：实现最小门禁**

```python
delay = _finite_number("start_delay_s", self.start_delay_s, minimum=0)
if delay > LIFT_TRIAL_S:
    raise CalibrationError("start_delay_s 不得超过完整起升试验周期")
```

analyze_lift_trials() 使用：

```python
if any(
    not trial.direction_consistent or trial.displacement_mm < 1.0
    for trial in trials
):
    raise CalibrationError(
        "40% PWM 的三次起升必须都同向且观察结束净位移至少 1 mm"
    )
if any(trial.start_delay_s > 0.3 for trial in trials):
    raise CalibrationError("起升响应延迟超过闭环允许的 300 ms")
if any(not trial.success for trial in trials):
    raise CalibrationError("起升试验成功标志与实测门禁不一致")
```

response_delay_s 仍取三次最大值，不截断实测数据。

- [ ] **步骤4：运行并确认GREEN**

```bash
python -m pytest -q tests/test_calibration.py -k "lift_analysis"
```

预期：全部通过。

- [ ] **步骤5：提交**

```bash
git add src/agv_lift_height_control/calibration.py tests/test_calibration.py
git commit -m "fix: validate settled lift response / 校验稳定后的起升响应"
```

### 任务2：实现一次预充压和三次完整周期测量

**文件：**

- 修改：tests/test_calibration.py
- 修改：src/agv_lift_height_control/calibration.py

- [ ] **步骤1：写现场复现测试**

测试依次驱动以下四个周期，每周期100 ms通电、700 ms全零：

```python
def test_lift_session_precharges_then_records_three_delayed_measurements() -> None:
    session = LiftCalibrationSession()

    run_lift_cycle(
        session,
        started_at=0.0,
        start_height=0.1,
        stop_height=0.1,
        first_motion_at=None,
        highest_height=0.1,
        final_height=0.1,
    )
    assert session.trials == ()
    release_lift_authorization(session, now=0.81, height=0.1)

    now = 0.82
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
```

run_lift_cycle() 必须用确定性时间戳调用 session.step()：起点授权为真；100 ms边界高度仍为 stop_height；first_motion_at 时高度为起点加0.2 mm；400 ms写入 highest_height；800 ms写入 final_height。所有观察阶段授权为假，所有返回命令必须为全零。

- [ ] **步骤2：写重新授权、方向和电流窗口测试**

新增三个独立用例：

1. 周期结束时授权仍为真，下一周期必须保持全零；先观察到假，再收到新的真才允许40%。
2. 停泵观察阶段高度低于起点0.5 mm以上，立即锁存“方向反向”并返回全零。
3. 观察阶段即使反馈电流绝对值更大，正式 LiftTrial.peak_current_raw 仍只取100 ms通电窗口峰值。

调整既有通电期撤权和观察期撤权测试：先完成预充压，再断言正式样本。

- [ ] **步骤3：运行并确认RED**

```bash
python -m pytest -q tests/test_calibration.py -k "lift_session"
```

预期：现有实现会把预充压保存成第1条样本、只在通电期寻找运动，并会借用旧授权。

- [ ] **步骤4：实现状态机**

增加：

```python
LIFT_PRECHARGE_REPEATS = 1
LIFT_MAX_RESPONSE_DELAY_S = 0.3

self._precharge_count = 0
self._awaiting_authorization_release = False
```

完成条件和释放门禁：

```python
@property
def done(self) -> bool:
    return (
        self._precharge_count >= LIFT_PRECHARGE_REPEATS
        and self._index >= LIFT_CALIBRATION_REPEATS
    )

if self._active_started_at is None and self._awaiting_authorization_release:
    if not lift_authorized:
        self._awaiting_authorization_release = False
    return PumpCommand.safe_stop()
```

完整周期机械观测与通电电流分离：

```python
def _observe_motion(self, now: float, height: float) -> None:
    self._lowest_height = min(self._lowest_height, height)
    if self._first_movement_at is None and height - self._start_height >= 0.1:
        self._first_movement_at = now


def _observe_powered(
    self, now: float, height: float, feedback: PumpFeedback | None
) -> None:
    self._observe_motion(now, height)
    if feedback is not None:
        self._peak_current = max(self._peak_current, abs(feedback.current_raw))
```

进入及处于观察阶段都调用 _observe_motion()；只有建立 _stop_height 后才用
max(self._highest_settle_height, height) 更新滑行最高点。当前周期处于活动状态时，
每个样本都先检查 height 是否低于起点方向容差，保证观察期也能立即失败。
_finish(final_height) 使用：

```python
displacement = final_height - self._start_height
direction_ok = self._lowest_height >= self._start_height - self._direction_tolerance_mm
delay = (
    self._first_movement_at - self._active_started_at
    if self._first_movement_at is not None
    else LIFT_TRIAL_S
)
if self._precharge_count < LIFT_PRECHARGE_REPEATS:
    self._precharge_count += 1
else:
    self._trials.append(
        LiftTrial(
            pwm=LIFT_CALIBRATION_PWM,
            repeat=self._index + 1,
            start_delay_s=max(delay, 0.0),
            displacement_mm=displacement,
            speed_mm_s=max(0.0, displacement) / LIFT_PULSE_S,
            coast_mm=max(0.0, self._highest_settle_height - self._stop_height),
            peak_current_raw=self._peak_current,
            direction_consistent=direction_ok,
            success=(
                direction_ok
                and displacement >= 1.0
                and self._first_movement_at is not None
                and delay <= LIFT_MAX_RESPONSE_DELAY_S
            ),
        )
    )
    self._index += 1
self._awaiting_authorization_release = True
self._reset_active()
```

预充压不生成 LiftTrial；首次运动搜索覆盖完整800 ms；延迟不截断。

- [ ] **步骤5：运行GREEN和控制器回归**

```bash
python -m pytest -q tests/test_calibration.py -k "lift_session or lift_analysis"
python -m pytest -q tests/test_controller.py tests/test_simulation.py
```

预期：全部通过，单档闭环仍不超过40%。

- [ ] **步骤6：提交**

```bash
git add src/agv_lift_height_control/calibration.py tests/test_calibration.py
git commit -m "fix: precharge before delayed lift trials / 延迟起升试验前先预充压"
```

### 任务3：升级起升草稿到schema v3

**文件：**

- 修改：tests/test_operator_runtime.py
- 修改：src/agv_lift_height_control/runtime_storage.py

- [ ] **步骤1：写v3往返和旧版拒绝测试**

```python
assert raw["schema_version"] == 3


@pytest.mark.parametrize("old_version", [1, 2])
def test_calibration_draft_rejects_old_lift_semantics(
    tmp_path, old_version: int
) -> None:
    path = tmp_path / "lift-draft.json"
    store = CalibrationDraftStore(path)
    store.save_lift(_lift_result())
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["schema_version"] = old_version
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(CalibrationError, match="旧版.*重新执行起升标定"):
        store.load_lift()
```

- [ ] **步骤2：运行并确认RED**

```bash
python -m pytest -q tests/test_operator_runtime.py -k "calibration_draft"
```

预期：当前保存版本仍为2，v2会被接受。

- [ ] **步骤3：实现严格v3**

```python
DRAFT_SCHEMA_VERSION = 3

schema_version = raw["schema_version"]
if type(schema_version) is not int:
    raise CalibrationError("不支持的起升草稿 schema_version")
if schema_version in {1, 2}:
    raise CalibrationError("旧版起升草稿字段语义已失效；请重新执行起升标定")
if schema_version != DRAFT_SCHEMA_VERSION:
    raise CalibrationError("不支持的起升草稿 schema_version")
```

保持精确字段集合、规范PWM键、摘要重算和原子保存不变。

- [ ] **步骤4：运行回归并提交**

```bash
python -m pytest -q tests/test_operator_runtime.py -k "draft or fingerprint"
python -m pytest -q tests/test_application_modes.py -k "confirm_lower or lift_draft"
git add src/agv_lift_height_control/runtime_storage.py tests/test_operator_runtime.py
git commit -m "fix: version delayed-response lift drafts / 升级延迟响应起升草稿版本"
```

预期：测试通过；新起升草稿产生新指纹，旧下降草稿仍被拒绝。

### 任务4：更新Orange Pi现场说明与维护追踪

**文件：**

- 修改：tests/test_repository_files.py
- 修改：README.md
- 修改：docs/维护地图.md

- [ ] **步骤1：写文档契约测试**

```python
assert "1 次预充压" in readme
assert "随后执行 3 次正式测量" in readme
assert "总共敲击 4 次 u" in readme
assert "观察结束净位移至少 1 mm" in readme
assert "响应延迟不得超过 300 ms" in readme
assert "schema v3" in readme
assert "以 5% 递增到 80%" not in readme

assert "LIFT_PRECHARGE_REPEATS" in maintenance
assert "完整 800 ms" in maintenance
assert "schema v3" in maintenance
assert "test_lift_session_precharges_then_records_three_delayed_measurements" in maintenance
```

- [ ] **步骤2：运行并确认RED**

```bash
python -m pytest -q tests/test_repository_files.py
```

预期：现有README和维护地图仍描述旧三次通电位移规则。

- [ ] **步骤3：更新README和维护地图**

README命令表改为“一次预充压＋三次正式40%短脉冲”。现场章节明确总共按四次u、每次等待约800 ms、预充压允许零位移、正式样本按最终净位移判断、响应延迟最大300 ms、失败保留CSV但不保存草稿、只接受schema v3。

维护地图增加 LIFT_PRECHARGE_REPEATS=1，明确 LIFT_CALIBRATION_REPEATS=3 只统计正式样本；LIFT_PULSE_S和LIFT_SETTLE_S覆盖完整800 ms响应搜索；DRAFT_SCHEMA_VERSION=3拒绝v1的27次数据和v2的通电位移数据；函数地图更新为“预充压→重新授权→三次正式测量”。

- [ ] **步骤4：运行GREEN并提交**

```bash
python -m pytest -q tests/test_repository_files.py
git add README.md docs/维护地图.md tests/test_repository_files.py
git commit -m "docs: explain four-step lift calibration / 说明四步起升标定"
```

预期：文档契约测试通过。

### 任务5：全量验证、审查并推送main

- [ ] **步骤1：运行全量门禁**

```bash
python -m pytest -q
python -m compileall -q src tests
git diff --check origin/main...HEAD
git status --short
```

预期：pytest全绿，compileall和diff-check退出码为0，工作区为空。

- [ ] **步骤2：核对提交范围**

```bash
git log --oneline origin/main..HEAD
git diff --stat origin/main...HEAD
git diff --name-only origin/main...HEAD
```

预期：只包含本次设计、计划、标定状态机、草稿schema、测试和中文文档；不包含kinco_duolun.py、本地配置、CSV或状态JSON。

- [ ] **步骤3：推送并核对远端**

```bash
git push origin main
git rev-parse HEAD
git ls-remote origin refs/heads/main
```

预期：远端和本地HEAD一致，然后给Orange Pi提供 git pull --ff-only 和四次按键命令。
