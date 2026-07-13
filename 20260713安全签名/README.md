# MAVLink 2 安全签名 · 第 4–5 周

本目录为实习 **安全签名阶段** 交付物：在 ArduPilot SITL 中启用 MAVLink 2 消息签名，并通过 Python 脚本验证「正确密钥 / 未配钥 / 错误密钥」三组对比实验。

综合实战项目（地理围栏）已独立至 [`../20260713地理围栏实战/`](../20260713地理围栏实战/)。

---

## 阶段目标

| 任务 | 脚本 | 状态 |
|------|------|------|
| MAVLink 2 安全签名启用与验证 | `secure_connection.py` | ✅ 已完成 |

---

## 目录结构

```
20260713安全签名/
├── README.md                          # 本说明
├── requirements.txt                   # Python 依赖
├── secure_connection.py               # 签名连接与三组对比验证
├── signing_output_20260713_094620.txt # 签名实验日志（样例）
├── MAVLink2安全签名报告.tex            # 安全签名报告（LaTeX）
└── MAVLink2安全签名报告.pdf            # 安全签名报告（PDF）
```

---

## MAVLink 2 签名原理（简述）

- MAVLink 2 签名是 **认证机制**，不是加密：遥测仍可被未授权方接收，但飞控 **只接受带正确签名的控制类命令**（参数读写、解锁、模式切换等）。
- 通信双方需共享同一个 **32 字节 secret key**。
- 飞控侧通过 MAVProxy `signing setup KEY` 配置；Python 侧通过 `master.setup_signing(key)` 配置。

---

## 环境要求

| 组件 | 说明 |
|------|------|
| OS | Windows 10 + Ubuntu VM（ArduPilot SITL） |
| Python | 3.10+ |
| 库 | `pymavlink >= 2.4.41` |
| 端口 | UDP **14551** |

---

## 快速开始

### 1. 启动 SITL（Ubuntu VM）

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --out=udp:<Windows主机IP>:14551
```

### 2. 飞控侧启用签名（MAVProxy 控制台）

在 `STABILIZE>` 提示符下执行：

```bash
module load signing
signing setup my_sitl_mavlink2_signing_key!!!!
```

### 3. Windows 运行验证脚本

```bash
cd "20260713安全签名"
pip install -r requirements.txt
python secure_connection.py --mode all
```

---

## `secure_connection.py` 用法

```bash
python secure_connection.py --mode signed      # 实验 A：正确密钥
python secure_connection.py --mode unsigned    # 实验 B：未配钥
python secure_connection.py --mode wrong-key   # 实验 C：错误密钥
python secure_connection.py --mode all         # 三组对比（推荐）
python secure_connection.py --fetch-timeout 60 # 增大参数拉取超时
```

| 参数 | 说明 | 默认 |
|------|------|------|
| `--key` | 与飞控一致的 32 字节密钥 | `my_sitl_mavlink2_signing_key!!!!` |
| `--wrong-key` | 错误密钥（实验 C） | `wrong_sitl_mavlink2_sign_key!!!!` |
| `--fetch-timeout` | 参数拉取超时（秒） | 12 |

---

## 验证结果（2026-07-13）

| 实验 | 参数响应 | 控制命令 | 结论 |
|------|----------|----------|------|
| A 正确密钥 | 1339 条完整 | ACK ACCEPTED | ✅ 符合预期 |
| B 未配钥 | 0 条 | 无 ACK | ✅ 被拒绝 |
| C 错误密钥 | 0 条 | 无 ACK | ✅ 被拒绝 |

详见 `signing_output_20260713_094620.txt`。

详细报告见 [`MAVLink2安全签名报告.pdf`](MAVLink2安全签名报告.pdf)。

---

## 常见问题

| 现象 | 处理 |
|------|------|
| 收不到 HEARTBEAT | SITL `--out` 须为 Windows IP:14551，不能用 `127.0.0.1` |
| 三组实验均成功 | 飞控未启用签名，需在 MAVProxy 执行 `signing setup` |
| 正确密钥只收到部分参数 | 增大 `--fetch-timeout`；签名本身已生效 |
| `allow_unsigned_callback` 报错 | 回调签名须为 `(mav, msgId) -> bool` |

实验结束后可在 MAVProxy 执行：

```bash
signing remove my_sitl_mavlink2_signing_key!!!!
```

---

## 参考

- [MAVLink Message Signing](https://mavlink.io/en/guide/message_signing.html)
- [ArduPilot MAVLink2 Signing](https://ardupilot.org/dev/docs/common-MAVLink2-signing.html)
- [pymavlink Message Signing](https://mavlink.io/en/mavgen_python/message_signing.html)
