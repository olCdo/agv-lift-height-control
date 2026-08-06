# CAN 泵安全层实施计划

> **给执行者：** 必须按测试驱动开发逐项执行本计划；本任务已由隔离工作树中的当前执行者直接完成，不派生子任务。

**目标：** 增加不改动系统网络配置、默认失效为全零输出的 CAN 泵通信安全层。

**架构：** `config.py` 负责强类型参数边界，`can_pump.py` 负责只读链路检查、协议纯函数、安全策略与双线程生命周期。硬件、副作用和时间全部经构造参数注入，单元测试使用完整的内存 CAN 消息与总线替身。

**技术栈：** Python 3.10、dataclass、threading、subprocess、python-can、pytest。

---

### 任务一：强类型 CAN 配置

**文件：**

- 修改：`src/agv_lift_height_control/config.py`
- 修改：`config/example.json`
- 测试：`tests/test_config.py`

- [ ] 先给 `valid_config()` 增加全部 CAN 字段，并新增正常加载、未知字段、类型、标准帧 ID、固定协议 ID、超时和停机帧数量的失败用例。
- [ ] 运行 `python -m pytest tests/test_config.py -q`，确认因 `CanConfig` 尚不存在或 `can` 仍为字典而红灯。
- [ ] 实现冻结的 `CanConfig`、`_parse_can()` 与带配置节名称的校验帮助函数；JSON 中缺少或多出字段都报 `ConfigError`。
- [ ] 更新示例 JSON，运行配置测试与 Task 1 全套测试确认绿灯。

### 任务二：只读链路检查和协议纯函数

**文件：**

- 创建：`src/agv_lift_height_control/can_pump.py`
- 测试：`tests/test_can_pump.py`

- [ ] 先写 `inspect_can_link()` 测试：只允许 `ip -details link show <interface>`，覆盖 UP、DOWN、bitrate 不符、命令失败和缺少 bitrate。
- [ ] 先写 `encode_pump_command()`、`encode_nmt_start()`、`parse_pump_feedback()` 测试，覆盖 DLC8 字段布局以及错误 ID、扩展/远程/错误帧、DLC 和字节边界。
- [ ] 运行目标测试，确认模块或接口缺失导致预期红灯。
- [ ] 最小实现 `CanLinkInfo`、`CanLinkError`、只读命令解析与纯协议函数；不调用任何 `ip link set`。
- [ ] 运行目标测试确认绿灯。

### 任务三：确定性安全策略与 CAN 生命周期

**文件：**

- 修改：`src/agv_lift_height_control/can_pump.py`
- 测试：`tests/test_can_pump.py`

- [ ] 先写纯 `select_safe_command()` 和 `CanPump.run_cycle()` 用例，覆盖 5 秒启动窗口、新鲜命令与反馈放行、命令过期、反馈缺失/过期、故障码、完整 8 字节周期帧和 NMT 仅在启动时发送。
- [ ] 先写生命周期用例：300ms 被动预检不发送、外部 0x217 冲突关闭、反馈接收更新、线程异常尝试归零、重复 stop 只关闭一次且发送指定数量零帧。
- [ ] 运行目标测试，确认行为缺失导致预期红灯。
- [ ] 实现可注入总线/消息/时钟/休眠/链路检查器的 `CanPump`，以锁保护 desired、反馈和故障；发送和接收线程均以失败即全零并停止为原则。
- [ ] 运行目标测试确认绿灯，再运行 `python -m pytest -q` 回归 Task 1。

### 任务四：公共接口、中文维护说明与提交

**文件：**

- 修改：`src/agv_lift_height_control/__init__.py`
- 修改：`src/agv_lift_height_control/can_pump.py`

- [ ] 从包根导出 `CanConfig`、链路检查、协议函数、策略和 `CanPump` 等稳定接口。
- [ ] 为预检、启动窗口、超时归零、线程异常和停机归零补充中文说明及关键安全注释。
- [ ] 运行 `python -m pytest -q`、`git diff --check`，复核配置变量到生效函数和测试的对应关系。
- [ ] 仅暂存本任务文件，检查 `git diff --cached`，提交一条同时含英文和中文的 commit message。
- [ ] 提交后再次运行全套测试并确认 `git status --short` 为空；明确记录尚未在真实 `can0` 硬件上验证。
