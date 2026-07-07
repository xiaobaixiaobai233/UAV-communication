# 深入 MAVLink 2 核心协议 · 第四阶段（第 4–5 周）

本目录为实习 **第四阶段** 交付物：通过 Python 脚本实现 MAVLink 2 参数、任务、命令等高级协议功能。脚本通过 UDP 14551 与 ArduPilot SITL 通信，**无需依赖 QGC 进行控制**。

---

## 阶段目标

> 通过脚本实现参数读写、任务上传与执行、命令控制等高级功能。

| 任务 | 脚本 | 协议 | 状态 |
|------|------|------|------|
| 任务 1 | `param_ops.py` | 参数协议（`PARAM_*`） | ✅ 已完成 |
| 任务 2 | `mission_upload.py` | 任务协议（`MISSION_*`） | ✅ 已完成 |
| 任务 3 | `command_control.py` | 命令协议（`COMMAND_LONG`） | ✅ 已完成 |

---

## 目录结构

```
20260707深入MAVLink 2核心协议/
├── README.md              # 本说明
├── requirements.txt       # Python 依赖
├── param_ops.py           # 任务 1：参数读写
├── mission_upload.py      # 任务 2：航线上传与执行
├── command_control.py     # 任务 3：命令控制
├── param_output_*.txt     # 任务 1 运行日志（自动生成）
├── mission_output_*.txt   # 任务 2 运行日志（自动生成）
└── command_output_*.txt   # 任务 3 运行日志（自动生成）
```

---

## 环境要求

| 组件 | 说明 |
|------|------|
| OS | Windows 10 + Ubuntu VM（ArduPilot SITL） |
| Python | 3.10+ |
| 库 | `pymavlink >= 2.4.41` |
| 仿真 | ArduPilot SITL（ArduCopter） |
| 端口 | UDP **14551**（脚本专用） |

前置环境搭建见上级目录 [`环境搭建操作步骤.txt`](../环境搭建操作步骤.txt) 与 [`20260701环境搭建和连接`](../20260701环境搭建和连接/)。

---

## 快速开始

### 1. 启动 SITL（Ubuntu 虚拟机）

只需向脚本端口输出，可不启动 QGC：

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --out=udp:<Windows主机IP>:14551
```

将 `<Windows主机IP>` 替换为 Windows 主机在桥接网络下的 IP（可用 `ipconfig` 查看）。

### 2. 安装依赖并运行（Windows）

```bash
cd "20260707深入MAVLink 2核心协议"
pip install -r requirements.txt
python param_ops.py
```

---

## 任务 1：`param_ops.py`

### 功能

1. 使用 `param_request_list()` 获取飞控全部参数
2. 按名称排序打印并写入日志文件
3. 读取并修改 1–2 个参数（默认 `SR1_EXT_STAT`、`SR1_EXTRA1`）
4. 通过 `param_request_read()` 读回，验证修改是否生效
5. 演示结束后默认**恢复原值**（避免污染 SITL 环境）

### 用法

```bash
# 完整演示：拉取全部参数 + 读写验证
python param_ops.py

# 仅拉取并列出参数，不修改
python param_ops.py --list-only

# 修改后保留新值
python param_ops.py --keep-changes

# 自定义要修改的参数
python param_ops.py --modify SR1_EXT_STAT=3 SR1_EXTRA1=5

# 指定日志文件
python param_ops.py --output my_param_log.txt
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--connection` | `udpin:0.0.0.0:14551` | MAVLink 连接字符串 |
| `--timeout` | `10` | 等待心跳超时（秒） |
| `--fetch-timeout` | `60` | 拉取全部参数超时（秒） |
| `--output` | 自动生成 | 日志文件路径 |
| `--list-only` | — | 只拉参数，不修改 |
| `--keep-changes` | — | 修改后不恢复原值 |
| `--modify` | `SR1_EXT_STAT=2 SR1_EXTRA1=4` | 要修改的参数（`NAME=VALUE`） |

### 典型输出

```
[连接] 已收到心跳 | target_system=1, target_component=0
[参数] 发送 param_request_list() 请求全部参数...
[参数] 飞控报告参数总数: 1200
[参数] 实际收到 1200 个参数
[演示] 参数 SR1_EXT_STAT: 当前值 = 0
[写入] param_set -> SR1_EXT_STAT = 2
[验证] SR1_EXT_STAT 读回成功，值与期望一致: 2
[演示] 参数已恢复为修改前的值
```

日志同时写入 `param_output_YYYYMMDD_HHMMSS.txt`。

> **日志文件提示**：若使用 `--output my_param_log.txt` 指定日志路径，请勿在 PyCharm 运行配置里把 **stdout 也重定向到同一文件**，否则可能造成日志开头重复。脚本已将终端输出改走 stderr，正常情况下可规避此问题。

### 默认修改的参数

| 参数名 | 含义 | 说明 |
|--------|------|------|
| `SR1_EXT_STAT` | Telem1 扩展状态流速率（Hz） | 任务要求示例参数 |
| `SR1_EXTRA1` | Telem1 姿态相关流速率（Hz） | 第二个演示参数 |

> **注意**：ArduPilot 中参数名为 `SR1_EXT_STAT`（带下划线），不是 `SR1_EXTSTAT`。`SRx_*` 流速率参数修改后，实际遥测推送频率通常需**重启飞控**才生效；脚本通过 `PARAM_VALUE` 读回确认**写入是否成功**。

---

## 任务 2：`mission_upload.py`

### 功能

1. 以当前 Home 为中心，构造 **4 个航点**：起飞 → 航点 A（北侧）→ 航点 B（东北侧）→ 返航（RTL）
2. 使用 `mission_count_send()` 告知航点数量
3. 响应 `MISSION_REQUEST` / `MISSION_REQUEST_INT`，分别用 `mission_item_send()` / `mission_item_int_send()` 上传
4. 等待 `MISSION_ACK` 确认上传成功
5. **STABILIZE 解锁 → 切 AUTO → MISSION_START**（AUTO 模式下不能直接解锁）
6. 监控航点进度与位置变化

### 用法

```bash
# 完整流程：上传 + 解锁 + 执行 + 监控
python mission_upload.py

# 仅上传航线，不执行（便于先验证上传是否成功）
python mission_upload.py --upload-only

# 自定义高度与航点偏移
python mission_upload.py --alt 15 --offset 40

# 自定义飞行/降落速度（单位 cm/s）
python mission_upload.py --wp-speed 400 --descent-speed 250 --land-speed 80

# 指定日志文件
python mission_upload.py --output mission_log.txt
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--connection` | `udpin:0.0.0.0:14551` | MAVLink 连接字符串 |
| `--timeout` | `10` | 等待心跳超时（秒） |
| `--alt` | `10` | 任务高度（米，相对 Home） |
| `--offset` | `30` | 航点 A/B 相对 Home 偏移（米） |
| `--upload-only` | — | 只上传，不解锁执行 |
| `--exec-timeout` | `300` | 任务执行监控超时（秒） |
| `--wp-speed` | `500` | 水平飞行速度 WPNAV_SPEED（cm/s，默认约为飞控一半） |
| `--descent-speed` | `220` | 下降/返航速度 WPNAV_SPEED_DN（cm/s） |
| `--land-speed` | `75` | 最终降落速度 LAND_SPEED（cm/s） |
| `--keep-speeds` | — | 任务结束后保留速度参数 |
| `--output` | 自动生成 | 日志文件路径 |

### 飞行参数

任务执行前会通过 `param_set` 临时调整以下参数，**任务结束后默认恢复原值**：

| 参数 | 默认值 | 飞控默认 | 作用 |
|------|--------|----------|------|
| `WPNAV_SPEED` | 500 cm/s (5 m/s) | 1000 | 航线水平飞行速度（略降） |
| `WPNAV_SPEED_DN` | 220 cm/s (2.2 m/s) | 150 | 下降/返航垂直速度（略升） |
| `LAND_SPEED` | 75 cm/s (0.75 m/s) | 50 | 最终接地降落速度（略升） |
| **`RTL_ALT`** | **与 `--alt` 相同** | **1500 (15m)** | **返航高度，自动与任务高度一致** |

> 若不设置 `RTL_ALT`，飞控返航时会先爬到默认 15m，导致相对高度从 10m 升到 15m。脚本现已将 `RTL_ALT` 设为 `--alt × 100`（cm）。

### 默认航线

ArduPilot 要求 **seq=0 为 Home 参考点**，实际飞行 4 步：起飞 → A → B → RTL（共上传 5 条 mission item）：

| seq | 航点 | 命令 | 说明 |
|-----|------|------|------|
| 0 | Home | `NAV_WAYPOINT` | Home 参考点（ArduPilot 协议要求） |
| 1 | 起飞 | `NAV_TAKEOFF` | 爬升至 `--alt` 米 |
| 2 | 航点 A | `NAV_WAYPOINT` | Home 北侧 `--offset` 米 |
| 3 | 航点 B | `NAV_WAYPOINT` | Home 东北侧（北+东各 `--offset` 米） |
| 4 | 返航 | `NAV_RETURN_TO_LAUNCH` | 自动返回起飞点 |

### 典型输出

```
[航线] 任务航点规划：
  seq=0  起飞      NAV_TAKEOFF               lat=..., lon=..., alt=10.0m
  seq=1  航点 A    NAV_WAYPOINT              lat=..., lon=..., alt=10.0m
  ...
[任务] mission_count_send() 航点数量 = 4
[上传] mission_item_int_send seq=0 (起飞, MAV_CMD_NAV_TAKEOFF)
...
[任务] 航线上传成功，共 4 个航点
[执行] 解锁成功
[执行] 当前模式: AUTO
[进度] 当前航点 seq=1/3
[进度] 返航完成，已上锁
```

### 注意事项

- 执行阶段脚本会**自动解锁并切 AUTO**，请勿与 QGC 同时控制
- 若仅想验证上传协议，先用 `--upload-only`
- 可在 QGC 地图（只读连接 14550）上观察飞机是否按航线飞行

---

## 任务 3：`command_control.py`

### 功能

1. 使用 `command_long_send()` 发送控制命令（**不上传航线**）
2. 完整流程：**解锁 → GUIDED → 起飞 10m → 返航（RTL）**
3. 每步等待 `COMMAND_ACK`，并通过 HEARTBEAT / 高度确认状态
4. 返航阶段监控直至上锁

### 用法

```bash
# 默认：起飞 10m，悬停 5s 后返航
python command_control.py

# 自定义起飞高度与悬停时间
python command_control.py --alt 12 --hold 8

# 指定日志文件
python command_control.py --output command_log.txt
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--connection` | `udpin:0.0.0.0:14551` | MAVLink 连接字符串 |
| `--timeout` | `10` | 等待心跳超时（秒） |
| `--alt` | `10` | 起飞高度（米） |
| `--hold` | `5` | 到达高度后悬停时间（秒） |
| `--exec-timeout` | `300` | 返航监控超时（秒） |
| `--keep-params` | 否 | 任务结束后保留 `RTL_ALT` 修改 |
| `--output` | 自动生成 | 日志文件路径 |

### 飞行参数

返航前脚本会将 **`RTL_ALT` 设为 `--alt × 100`（cm）**，与任务 2 相同：

| 参数 | 脚本设置 | 飞控默认 | 说明 |
|------|----------|----------|------|
| **`RTL_ALT`** | **与 `--alt` 相同** | **1500 (15m)** | **返航高度，避免 RTL 时先爬升再下降** |

> 若不设置 `RTL_ALT`，返航时会先爬到默认 15m。任务 3 日志中可见悬停 ~10m 后 RTL 升至 ~12m 再下降；设置后应保持在 `--alt` 附近直接降落。

### 命令序列

| 步骤 | 命令 | API |
|------|------|-----|
| 1 | 解锁 | `MAV_CMD_COMPONENT_ARM_DISARM` (param1=1) |
| 2 | 切 GUIDED | `set_mode_apm('GUIDED')` / `DO_SET_MODE` |
| 3 | 起飞 | `MAV_CMD_NAV_TAKEOFF` (param7=高度) |
| 4 | 返航 | `MAV_CMD_NAV_RETURN_TO_LAUNCH` |

### 典型输出

```
[步骤 1] 解锁（Arm）
[命令] MAV_CMD_COMPONENT_ARM_DISARM COMMAND_ACK | result=ACCEPTED
[步骤 2] 切换 GUIDED 模式
[状态] 当前模式: GUIDED
[步骤 3] 起飞至 10.0 米
[状态] 已到达目标高度 | relative_alt=10.02m
[步骤 4] 返航（RTL）
[进度] 返航完成，已上锁
```

### 与任务 2 的区别

| | 任务 2 `mission_upload.py` | 任务 3 `command_control.py` |
|--|---------------------------|----------------------------|
| 协议 | `MISSION_*` | `COMMAND_LONG` |
| 模式 | AUTO | GUIDED |
| 路径 | 预上传航点 A/B | 原地起飞 → 返航 |

---

## 与 QGC 的关系

| 阶段 | QGC 角色 |
|------|----------|
| 第 2–3 周（`basic_communication.py`） | 必需：解锁、起飞、画航线 |
| **本阶段** | **不必需**：脚本独立完成参数/任务/命令操作 |

调试时仍可打开 QGC 在地图上观察飞机状态，但请避免与脚本同时发送控制指令。

---

## 常见问题

| 问题 | 处理 |
|------|------|
| 连接超时，无 HEARTBEAT | 确认 SITL 已启动且 `--out` 包含 Windows IP 与端口 14551 |
| 拉取参数超时 | 增大 `--fetch-timeout`；ArduPilot 参数较多，首次拉取可能需要数十秒 |
| 参数名不存在 | 用 `--list-only` 查看完整列表，确认参数名拼写 |
| 写入成功但遥测频率未变 | `SRx_*` 参数需重启 SITL 后才会影响实际推送频率，属正常现象 |
| 航线上传超时 | 确认 SITL 正常；可重试；检查是否有 QGC 同时在上传任务 |
| 解锁失败 | **AUTO 模式不可解锁**；脚本已改为 STABILIZE 解锁后再切 AUTO；请断开 QGC 避免抢控制权 |
| AUTO 模式切换失败 / Missing Takeoff Cmd | 确认 seq=0 为 Home、seq=1 为 TAKEOFF（ArduPilot 协议要求）；查看 `[飞控]` 日志 |
| GUIDED 起飞失败 / AUTO mode not armable | 脚本会自动先切 STABILIZE；若刚跑过 `mission_upload.py`，请断开 QGC 或重启 SITL |

---

## 后续任务预览

第四阶段三项脚本（参数 / 任务 / 命令）已全部完成。后续可进行 **MAVLink 2 安全签名** 与 **地理围栏监测** 最终目标开发。

---

## 参考

- [MAVLink Parameter Protocol](https://mavlink.io/en/services/parameter.html)
- [ArduPilot: Getting and Setting Parameters](https://ardupilot.org/dev/docs/mavlink-get-set-params.html)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
- [ArduPilot: Missions](https://ardupilot.org/copter/docs/common-planning-a-mission-with-waypoints-and-events.html)
