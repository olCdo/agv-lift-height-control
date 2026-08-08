# AGV 升降闭环测试程序

这是一个面向 Orange Pi 5 Plus 与 Ubuntu 的前台测试程序，用 Modbus RTU 拉绳位移传感器读取升降高度，并通过 `can0` 控制液压泵。首版用于现场标定、低高度闭环定高、人工下降和安全上限测量。

程序只自动起升，不会为了修正超调而自动下降。人工下降必须持续通过 SSH 键盘续期；停止按键、SSH 断开、输入结束、信号、线程异常或安全门禁失败都会撤销授权并请求发送完整全零命令。

> [!CAUTION]
> 本项目属于会驱动物理机构的测试软件，不是经过功能安全认证的控制器。第一次运行时，操作者必须在设备旁监护，确认急停和独立物理断电有效，并能立即切断动力。测试期间必须停止 OpenPLC 和现有车辆驱动，禁止与 `kinco_duolun.py` 或其他发送 `0x217` 的程序同时运行。

## 当前实现范围

- 高度传感器：默认按 BRT38-3M-R0M4096-RT1-IP68-ZJ 的 3000 mm 规格配置，使用 Modbus RTU、FC03、Slave ID 3，从 HR0 连续读取两个寄存器。
- 默认组合：`raw = (HR0 << 16) | HR1`，高度为 `raw × 200 / 4096 mm`。
- 默认量程：3000 mm，对应有效计数 `0..61440`；程序不会自动写零点。
- 泵命令：标准帧 `0x217`，每 50 ms 发送一次完整 8 字节命令。
- 泵反馈：标准帧 `0x197`；反馈超过 150 ms、故障码非零或没有反馈时停机。
- 启动：先被动监听 300 ms，检测到其他发送者的 `0x217` 就拒绝使能；随后保持 5 秒 NMT/全零安全窗口。
- 控制：粗升、实测档位受限 P 控制、末端脉冲和保持；目标误差要求连续 500 ms 位于 ±2 mm。
- 超调：超过目标立即归零，超过 5 mm 判本次测试失败，禁止自动下降修正。
- 上限：程序绝对保护不允许超过 2900 mm；没有持久软限位时，起升相关命令必须人工输入临时上限。
- SSH：只允许真实前台终端，拒绝 `tmux`、`screen`、`nohup`、后台进程和 `TERM=dumb`。

当前没有实现 Modbus TCP。控制器、标定器和运行循环只依赖 `HeightSource` 公共接口，后续可以新增 TCP 高度源，不需要把 RTU 细节写进闭环状态机。

## 目录

```text
config/example.json                 严格配置示例
src/agv_lift_height_control/        生产代码
tests/                              单元、状态机、仿真和运行时测试
docs/外部命令安全仲裁.md             首次标定与重标定的安全边界
docs/维护地图.md                     模块、变量、调用链和测试索引
```

## Orange Pi 安装

要求 Python 3.10 或更高版本。以下命令在 Orange Pi 的普通 SSH 前台会话中执行：

```bash
sudo apt update
sudo apt install -y python3-venv can-utils

git clone https://github.com/olCdo/agv-lift-height-control.git
cd agv-lift-height-control
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
cp config/example.json config/local.json
python -m pytest -q
```

`config/local.json`、标定草稿、最终标定、CSV 日志和 `.venv` 都不会提交到 Git。

以后更新固定工作目录时，先停止测试程序，再执行：

```bash
git pull --ff-only
. .venv/bin/activate
python -m pip install -e ".[dev]"
python -m pytest -q
```

本项目不配置自动部署、后台服务或开机启动。每次测试都由操作者检查现场后手动启动。

## 确认传感器设备名

默认配置使用 `/dev/ttyS7`。不要因为 OpenPLC 中出现过类似 `ttyUSBs7` 的名字就直接照抄；先在 Orange Pi 上确认内核实际设备节点：

```bash
ls -l /dev/ttyS7 /dev/ttyUSB* 2>/dev/null
readlink -f /dev/ttyS7
udevadm info -q property -n /dev/ttyS7
```

如果实际是 USB 转串口设备，把 `config/local.json` 的 `sensor.port` 改成真实路径。串口默认参数是 115200、8N1、自动收发 RS485 模块、Slave ID 3。需要串口权限时：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录 SSH 后权限才会生效。程序读取 FC03 的两个保持寄存器，地址、数量、字序、轮周长和每圈计数都来自 JSON；更换传感器或 HR 地址时只改配置，不改闭环代码。

## 准备并检查 CAN

程序不会修改系统网络配置。操作者负责把 `can0` 配为 500 kbit/s：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

开始测试前，人工确认 OpenPLC、现有车辆驱动和其他泵控制程序已经停止：

```bash
pgrep -afi 'openplc|kinco_duolun|agv_lift_height_control'
```

这个命令只检查，不会结束进程。本项目的单实例锁只能阻止本项目自身重复运行；启动前的 300 ms 监听只能发现当时正在发送的外部 `0x217`，不能替代人工停用其他控制器。

## 前台操作键

| 按键 | 行为 |
| --- | --- |
| `u` | 起升授权续期 700 ms，只在允许起升的模式生效 |
| `d` | 下降授权续期 150 ms，只在允许下降的模式生效 |
| `c` | 请求清除可恢复的控制器故障；不会绕过仍存在的故障条件 |
| `q` | 撤销全部授权、请求全零并安全退出 |

按键采用续期而不是切换。必须持续重复按下对应按键；停止重复后授权自动过期。不要通过 `tmux`、`screen`、`nohup`、`systemd` 或 shell 后台运行。

## 命令

所有命令都使用同一个严格 JSON 配置：

```bash
CONFIG_PATH=config/local.json
python -m agv_lift_height_control --config "$CONFIG_PATH" --help
```

| 命令 | 是否打开传感器 | 是否打开 CAN | 说明 |
| --- | --- | --- | --- |
| `monitor` | 是 | 否 | 默认只读高度 60 秒 |
| `observe-can` | 否 | 仅接收 | 默认被动观察 `0x197` 60 秒，不发送帧 |
| `zero-can` | 否 | 是 | 只运行 NMT 和 `0x217` 全零窗口 |
| `calibrate-lift` | 是 | 是 | 1 次预充压加 3 次固定 40% 正式测量，强制输入临时最大高度，完成后保存起升草稿 |
| `prepare-lower` | 是 | 是 | 读取起升草稿，以固定 40% 短脉冲预升到指定传感器高度，不写标定文件 |
| `calibrate-lower` | 是 | 是 | 使用当前起升草稿完成下降动作测量，保存下降草稿 |
| `confirm-lower` | 否 | 否 | 根据已保存实测候选确认舒适下降阀值并生成最终标定包 |
| `move` | 是 | 是 | 只自动起升到目标高度；没有持久软限位时还要给临时上限 |
| `manual-lower` | 是 | 是 | 仅在 `d` 死手授权期间人工下降 |
| `survey-upper` | 是 | 是 | 用最低稳定 PWM 和最长 1 秒连续授权测量安全上限，保存测量草稿 |
| `confirm-upper` | 否 | 否 | 从测量草稿确认不高于安全建议的持久软上限 |
| `show-calibration` | 否 | 否 | 输出最终标定 JSON |

`confirm-lower` 和 `confirm-upper` 是动作后的独立确认命令。它们不会打开 TTY、串口或 CAN；草稿通过完整标定指纹绑定，防止把不同轮次的结果混在一起。

## 首次现场门禁

必须按顺序执行，任一步异常都先停止并排查，不得跳过：

程序显示的高度是从拉绳传感器零点开始的**相对起升行程**，仍按
`raw × 200 / 4096` 换算；它不是平台离地绝对高度，不叠加 96.5 mm 等机械安装偏置。

### 1. 只读高度 60 秒

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" monitor --duration-s 60
```

确认原始计数与高度连续、方向正确、静止时不跳变，并核对实际串口设备名、HR 地址和字序。

### 2. 被动观察泵反馈

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" observe-can --duration-s 60
```

这一模式不会发送任何 CAN 帧。确认能持续收到 `0x197`，故障码和电流字段合理。

### 3. 只发送全零

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" zero-can --duration-s 5
```

确认机构完全不动作，并确认总线上没有其他 `0x217` 发送者。

### 4. 起升标定

先人工确定本次测试绝不能超过的临时高度，再执行：

```bash
read -r -p "输入本次起升标定临时最大高度(mm): " LIFT_TEMP_MAX_MM
python -m agv_lift_height_control --config "$CONFIG_PATH" calibrate-lift \
  --temporary-max-mm "$LIFT_TEMP_MAX_MM"
```

首版先执行 1 次预充压，随后执行 3 次正式测量，四次都固定为 40% PWM，不会自动升到
45%～80%。每次通电 100 ms，随后保持 CAN 全零观察 700 ms；预充压和三次正式测量
全部完成后才结束标定。临时上限、已有持久软限位、配置绝对上限和 2900 mm 中的
最小值生效。

每次准备动作时敲击一次 `u` 后松开，总共敲击 4 次 `u`。单个字符的 700 ms 授权足以
覆盖 100 ms 通电，观察期可以松开且不会丢弃已经完成的脉冲。不要在 700 ms 观察期
持续重复 `u`；等待本次约 800 ms 的完整周期结束后，再为下一次动作重新敲击。程序
必须先观察到授权释放，才接受下一次动作。若 100 ms 通电阶段授权提前失效，本次
不完整周期会丢弃并保持全零。

预充压允许零位移且不计入正式样本。三次正式测量都要求方向一致、观察结束净位移至少 1 mm，
并且从通电开始到首次可测运动的响应延迟不得超过 300 ms；任一次不满足时本次
标定失败，不会自动提高 PWM。停泵后的最大上滑仍完整记录，用于闭环提前减速。CSV
即使中断或失败也会保留；只有三次正式测量全部成功才写入
`lift-calibration-draft.json`。当前只接受 schema v3 起升草稿；旧版 schema v1 的 27 次
动作数据和 schema v2 的通电位移数据都不会被静默复用，更新后必须按本节重新标定。

### 5. 为下降标定安全预升

设备无法通过独立手动控制起升时，先使用已经通过的 schema v3 起升草稿准备下降行程。本次现场目标为传感器相对高度 100 mm，临时最大高度仍为 200 mm：

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" prepare-lower \
  --target-mm 100 \
  --temporary-max-mm 200
```

程序只使用草稿里的最低稳定 PWM（本次为 40%）。初始 5 秒 NMT 安全窗口保持全零；窗口结束且 `0x197` 新鲜无故障后，持续按住 `u` 才会推进动作。每次通泵 100 ms，随后强制保持全零观察 700 ms；即使通泵阶段授权中断，也必须完成观察期，不能靠快速重复按键拼成长时间通泵。

高度达到 100 mm 后程序立即请求全零并安全退出，不会自动下降，也不会尝试保持在 100 mm。液压停泵后仍可能继续上滑，最终高度可能高于目标但不应超过起升草稿记录的最大上滑量；本次实测最大上滑约 4.297 mm。程序还要求目标加最大上滑不超过临时上限、已有持久软限位、配置绝对上限和 2900 mm 的最小值。

该命令不会改写 `lift-calibration-draft.json`，不会生成下降草稿。运行前必须停止 OpenPLC 和其他 `0x217` 发送者，操作者必须在设备旁并能立即物理断电。

### 6. 下降动作标定与事后确认

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" calibrate-lower
```

操作者持续按 `d`。程序从 `0x10` 到 `0xA0` 逐级测量，动作结束后保存下降草稿并打印成功候选。观察结果后，再从成功实测候选中确认舒适值，例如：

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" confirm-lower \
  --comfortable-valve 0x40
python -m agv_lift_height_control --config "$CONFIG_PATH" show-calibration
```

示例中的 `0x40` 不能照搬，必须选择本次输出的成功实测值。起升草稿被覆盖后，旧下降草稿会因指纹不匹配而拒绝确认。

### 7. 三次低高度闭环定高

没有持久软限位时，每次 `move` 都必须提供临时上限：

```bash
read -r -p "输入本次闭环测试临时最大高度(mm): " MOVE_TEMP_MAX_MM
python -m agv_lift_height_control --config "$CONFIG_PATH" move \
  --target-mm 100 --temporary-max-mm "$MOVE_TEMP_MAX_MM"
```

持续按 `u` 授权，只选择已确认安全的低高度目标并至少重复三次。首版闭环的粗升、P 区和
末端脉冲都只会发送 40% PWM。从连续起升进入末端区时，控制器先发送全零并完成一次
停泵观察，再从静止状态发出固定 100 ms 短脉冲；实测响应延迟只延长全零观察时间，
不会延长通泵时间。进入目标 ±2 mm 稳定窗口后，正常保持帧为
`217#0100000000000000`；连续 500 ms 位于窗口内后进入 `hold`。故障、超时、退出和命令
过期仍发送 `217#0000000000000000`。TUI 的“互锁=开/关”对应 Byte0。超过目标 2 mm
会判本次失败并保持全零，超过 5 mm 会锁存故障。若需要下降，退出后单独运行
`manual-lower`，不要期待自动控制回降。

### 8. 测量最高安全高度

```bash
read -r -p "输入本次上限测量临时最大高度(mm): " SURVEY_TEMP_MAX_MM
python -m agv_lift_height_control --config "$CONFIG_PATH" survey-upper \
  --temporary-max-mm "$SURVEY_TEMP_MAX_MM"
```

操作者持续按 `u`，程序只用最低稳定 PWM，单次连续起升不超过 1 秒。完成后程序保存测量草稿并输出建议软上限：

`最高观测高度 - max(50 mm, 2 × 最大滑行距离)`。

### 9. 确认持久软上限

人工审核测量结果后，选择一个不高于程序安全建议的具体值：

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" confirm-upper \
  --soft-limit-mm 900
python -m agv_lift_height_control --config "$CONFIG_PATH" show-calibration
```

示例中的 900 mm 不能照搬。最终值还会受到配置绝对上限和 2900 mm 保护；测量后如果标定包被修改，旧上限草稿会拒绝确认。

## 人工下降

```bash
python -m agv_lift_height_control --config "$CONFIG_PATH" manual-lower
```

只有持续按 `d` 时下降阀才会打开，每个按键事件只续期 150 ms。松开、停止重复、输入结束或 SSH 断开都会关阀并进入安全退出。

## 故障与停机条件

以下任一条件都会禁止动作并请求全零；部分故障会锁存，排除原因后才可尝试按 `c`：

- RTU 样本无效或超过 100 ms；
- 相邻高度速度超过 1.2 m/s、时间戳回退或同时间戳出现不同高度；
- 实际运动方向与命令累计不一致；
- 控制循环超过 100 ms；
- `0x197` 缺失、超过 150 ms、时间戳异常或反馈故障码非零；
- CAN 发送/接收线程异常、命令超过 150 ms 未刷新；
- 高度越过临时上限、持久软限位或 2900 mm 绝对保护；
- 同 PWM 泵电流超过标定峰值 1.5 倍并持续 200 ms；
- 目标超调超过 5 mm；
- 外部进程在启动预检期间发送 `0x217`；
- SIGHUP、SIGTERM、SIGINT、Ctrl+C、`q`、stdin EOF 或正常退出。

CSV 同时记录控制器期望命令、最后成功发送的实际命令和是否已经请求归零。看到“归零请求”不等于归零帧已经成功发送；退出事件会在停泵补发零帧后记录最后成功发送值。物理急停和独立断电始终是最终保护。

## 状态与日志

默认保存到 `~/.local/state/agv-lift-height-control/`：

```text
calibration.json                 最终标定与持久软限位
lift-calibration-draft.json      起升标定草稿
lower-calibration-draft.json     绑定起升指纹的下降草稿
upper-survey-draft.json          绑定最终标定指纹的上限测量草稿
agv-lift-height-control.lock     单实例锁
logs/*.csv                       周期、授权、故障和退出事件
```

草稿和最终标定使用严格、版本化 JSON。未知字段、旧的无指纹下降草稿、非规范键或指纹不匹配都会被拒绝，程序不会猜测兼容。

## 后续迁移到 Modbus TCP

v1 的 `sensor.transport` 只能是 `rtu`，这是有意的严格门禁。迁移时应：

1. 新增实现相同 `HeightSource.open()/read_sample()/close()` 的 TCP 高度源；
2. 给配置增加明确的 RTU/TCP 分支与各自必需字段，继续拒绝未知字段；
3. 在应用依赖工厂中按传输类型选择高度源；
4. 为新的 HR 地址、寄存器数量、字序和比例增加测试；
5. 保持 `HeightSample`、控制器、标定器、TUI 和 CAN 泵接口不变。

不要为了 TCP 直接放宽现有严格配置，否则拼写错误可能被静默当成默认值。

## 开发验证

```bash
python -m pip install -e ".[dev]"
python -m pytest -q
python -m compileall -q src tests
```

测试覆盖寄存器组合、pymodbus 3.x API 兼容、CAN 编解码与调度、控制状态机、液压延迟仿真、SSH 授权、信号/EOF 停机、外部 `0x217` 冲突、版本化草稿和无硬件确认路径。GitHub Actions 在 Python 3.10 与 3.12 上运行同一组测试。

单元和仿真测试通过不代表实机验证已经完成。首次 Orange Pi、RS485、SocketCAN、SSH 断线和液压响应仍必须严格按现场门禁逐项确认。

## 维护

修改控制或安全逻辑前先阅读 [维护地图](docs/维护地图.md) 和 [外部命令安全仲裁](docs/外部命令安全仲裁.md)。本项目采用 MIT 许可证，见 [LICENSE](LICENSE)。
