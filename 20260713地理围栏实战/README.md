# 地理围栏监测 · 综合实战项目

独立实战项目：实时监测无人机位置，当进入指定圆形围栏区域时，自动悬停并反向飞出至围栏边界。

---

## 项目目的

将前序阶段所学的 MAVLink 连接、位置解析、模式切换与导航命令整合为一个 **机载逻辑应用**：

1. 从 Home 点起飞并向北飞行；
2. 实时监测 `GLOBAL_POSITION_INT`；
3. 进入围栏后触发 **LOITER 悬停 → GUIDED 反向飞出 → 边界 LOITER**。

---

## 围栏设定（默认）

| 参数 | 值 |
|------|-----|
| 圆心 | Home 点正北 **100 m** |
| 半径 | **30 m** |
| 进入边界 | Home 北 **70 m** |
| 退出点 | 圆心正南 30 m（Home 北 **70 m**，南边界） |
| 北飞测试目标 | Home 北 **115 m**（围栏内部） |
| 飞行高度 | **10 m** |

```
Home ──70m──┤进入边界├── 圆心(100m) ──115m── 北飞目标
              └──── 半径 30m 的圆 ────┘
```

---

## 目录结构

```
20260713地理围栏实战/
├── README.md                              # 本说明
├── requirements.txt                       # Python 依赖
├── geofence_monitor.py                    # 主脚本
├── geofence_output_20260713_100424.txt    # SITL 验证日志（样例）
├── 地理围栏实战项目报告.tex                # 项目报告（LaTeX）
└── 地理围栏实战项目报告.pdf                # 项目报告（PDF）
```

---

## 环境要求

| 组件 | 说明 |
|------|------|
| OS | Windows 10 + Ubuntu VM（ArduPilot SITL） |
| Python | 3.10+ |
| 库 | `pymavlink >= 2.4.41` |
| 端口 | UDP **14551**（勿与 QGC 同时控制） |

---

## 快速开始

### 1. 启动 SITL

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --out=udp:<Windows主机IP>:14551
```

等待 MAVProxy 出现 `STABILIZE>` 且 GPS/EKF 就绪（约 15–20 秒）。

### 2. 运行脚本（Windows）

```bash
cd "20260713地理围栏实战"
pip install -r requirements.txt
python geofence_monitor.py
```

---

## 命令行参数

```bash
python geofence_monitor.py --alt 10 --center-north 100 --radius 30
python geofence_monitor.py --fly-target-north 115 --nav-wait 60
python geofence_monitor.py --output my_geofence_log.txt
```

| 参数 | 说明 | 默认 |
|------|------|------|
| `--alt` | 飞行高度（m） | 10 |
| `--center-north` | 圆心距 Home 北向距离（m） | 100 |
| `--radius` | 围栏半径（m） | 30 |
| `--fly-target-north` | 北飞测试目标（m） | 115 |
| `--nav-wait` | GPS/EKF 就绪等待超时（s） | 45 |
| `--hover-in-fence` | 进入围栏后悬停秒数 | 3 |
| `--hover-at-edge` | 边界悬停秒数 | 8 |

---

## 自动流程

```
连接 SITL → 等待 GPS/EKF → 解锁 → GUIDED → 起飞 10m
    → 向北飞 → 监测进入围栏（距圆心 ≤ 30m）
    → LOITER 悬停 → GUIDED 飞向退出点 → LOITER 边界悬停
```

---

## 验证结果（2026-07-13）

日志 `geofence_output_20260713_100424.txt` 摘录：

```
[触发] 进入围栏！| 距圆心=29.3m <= 半径30m
[响应] 围栏内悬停（LOITER）...
[响应] 反向飞向围栏南边界...
[状态] 到达 围栏南边界 | dist=1.9m
[完成] 围栏触发响应流程结束，飞机在南边界悬停
```

运行时长约 47 秒，全流程在 SITL 中端到端通过。

详细报告见 [`地理围栏实战项目报告.pdf`](地理围栏实战项目报告.pdf)。

---

## 常见问题

| 现象 | 处理 |
|------|------|
| GUIDED 切换失败 `requires position` | SITL 启动后多等 15–20s，或 `--nav-wait 60` |
| PreArm: EKF3 waiting for GPS | 同上，脚本已内置 GPS/EKF 等待 |
| 未进入围栏 | 增大 `--fly-target-north`，确保目标在圆内 |
| 与 QGC 冲突 | 运行脚本时断开 QGC |

---

## 相关阶段

- 安全签名：[`../20260713安全签名/`](../20260713安全签名/)
- 第四阶段协议脚本：[`../20260707深入MAVLink 2核心协议/`](../20260707深入MAVLink%202核心协议/)
