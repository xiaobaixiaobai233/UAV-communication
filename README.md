# UAV Communication · MAVLink Internship Project

A systematic internship project for **MAVLink 2** drone communication: from protocol research and environment setup, through Python interaction and advanced protocol control, to MAVLink 2 signing verification and a geofence monitoring demo.

**Stack:** Windows 10 + Ubuntu VM (ArduPilot SITL) + QGroundControl + pymavlink

> 📄 **Full project report:** [`MAVLink实习项目总报告.pdf`](MAVLink实习项目总报告.pdf) (10 pages, all phases)

---

## Table of Contents

- [Progress](#progress)
- [Repository Structure](#repository-structure)
- [System Architecture](#system-architecture)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Scripts by Phase](#scripts-by-phase)
- [Deliverables by Phase](#deliverables-by-phase)
- [Reports](#reports)
- [FAQ](#faq)
- [References](#references)

---

## Progress

| Phase     | Timeline   | Task                                                 | Key Deliverable                    | Status |
| --------- | ---------- | ---------------------------------------------------- | ---------------------------------- | ------ |
| Week 1    | 2026-07-01 | MAVLink research + env setup + QGC–SITL verification | Environment setup report           | ✅ Done |
| Weeks 2–3 | 2026-07-03 | Python & MAVLink 2 basic interaction                 | `basic_communication.py`           | ✅ Done |
| Weeks 3–4 | 2026-07-07 | Parameter / mission / command protocols              | `param_ops.py` and related scripts | ✅ Done |
| Weeks 4–5 | 2026-07-13 | MAVLink 2 message signing                            | `secure_connection.py`             | ✅ Done |
| Final     | 2026-07-13 | Geofence monitoring demo                             | `geofence_monitor.py`              | ✅ Done |

**Capability evolution:** passive listening → active control → security signing → application demo

---

## Repository Structure

```
无人机通信/
├── README.md
├── 任务.txt                               # Internship task schedule
├── 环境搭建操作步骤.txt                    # Environment setup checklist
├── MAVLink实习项目总报告.tex / .pdf        # Full project report
│
├── 20260701环境搭建和连接/                 # Week 1
│   ├── MAVLink环境搭建与连接验证报告.tex
│   └── screenshots/
│
├── 20260703Python与MAVLink 2基础交互/      # Weeks 2–3
│   ├── basic_communication.py
│   ├── requirements.txt
│   ├── mavlink_output_*.txt
│   └── Python与MAVLink2基础交互汇报.tex
│
├── 20260707深入MAVLink 2核心协议/          # Weeks 3–4
│   ├── README.md
│   ├── param_ops.py
│   ├── mission_upload.py
│   ├── command_control.py
│   ├── requirements.txt
│   ├── *_output_*.txt
│   └── 深入MAVLink2核心协议汇报.tex
│
├── 20260713安全签名/                       # Weeks 4–5: signing
│   ├── README.md
│   ├── secure_connection.py
│   ├── requirements.txt
│   ├── signing_output_*.txt
│   └── MAVLink2安全签名报告.tex / .pdf
│
└── 20260713地理围栏实战/                   # Final demo
    ├── README.md
    ├── geofence_monitor.py
    ├── requirements.txt
    ├── geofence_output_*.txt
    └── 地理围栏实战项目报告.tex / .pdf
```

> External tools (QGroundControl, ArduPilot source, etc.) are not included due to size. Install them locally following the documentation.

---

## System Architecture

### Topology

```
┌─────────────────────────────────────────────────────────────┐
│  Ubuntu VM (VMware bridged mode)                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ArduPilot SITL (ArduCopter)                        │    │
│  │  sim_vehicle.py --out=udp:<WindowsIP>:14550           │    │
│  │                 --out=udp:<WindowsIP>:14551           │    │
│  └──────────────┬──────────────────────┬───────────────┘    │
└─────────────────┼──────────────────────┼────────────────────┘
                  │ MAVLink 2 / UDP      │ MAVLink 2 / UDP
                  ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│  Windows 10 Host                                            │
│  ┌──────────────────────┐   ┌─────────────────────────────┐ │
│  │  QGroundControl      │   │  Python scripts (pymavlink) │ │
│  │  UDP 14550           │   │  UDP 14551                  │ │
│  └──────────────────────┘   └─────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Ports

| Port      | Client         | Purpose                                                 |
| --------- | -------------- | ------------------------------------------------------- |
| **14550** | QGroundControl | Ground-station control and monitoring (Weeks 2–3)       |
| **14551** | Python scripts | Data logging / active control / signing / geofence demo |

### Design Notes

- **Weeks 2–3:** QGC controls the vehicle on 14550; `basic_communication.py` **passively listens** on 14551
- **Weeks 3–4 onward:** Python scripts **actively control** the vehicle on 14551; QGC is not required
- **Important:** Do not run QGC and control scripts at the same time to avoid command conflicts
- **SITL output address:** Must use the Windows bridged IP (`ipconfig`), **not `127.0.0.1`**

---

## Requirements

| Component       | Version / Notes                                      |
| --------------- | ---------------------------------------------------- |
| Host OS         | Windows 10                                           |
| VM              | VMware Workstation + Ubuntu (**bridged networking**) |
| Python          | 3.10+                                                |
| Ground station  | QGroundControl 4.2+ (MAVLink 2 support)              |
| Flight sim      | ArduPilot SITL (ArduCopter, Copter-4.4 branch)       |
| Python library  | `pymavlink >= 2.4.41`                                |
| Version control | Git 2.47+                                            |

For detailed setup, see [`环境搭建操作步骤.txt`](环境搭建操作步骤.txt) and [`20260701环境搭建和连接/`](20260701环境搭建和连接/).

---

## Quick Start

### 0. Get Windows IP

In PowerShell:

```powershell
ipconfig
```

Find the IPv4 address on the VMware bridged adapter (e.g. `192.168.112.233`). Use it as `<WindowsIP>` below.

### 1. Start SITL (Ubuntu VM)

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --out=udp:<WindowsIP>:14551
```

Add `:14550` if QGC is needed. Wait for the `STABILIZE>` prompt in MAVProxy. For control scripts (geofence, etc.), wait an additional **15–20 seconds** for GPS/EKF to become ready.

### 2. Install Python Dependencies (Windows)

Each phase directory includes `requirements.txt`:

```bash
pip install -r requirements.txt
```

### 3. Run Scripts by Phase

#### Weeks 2–3 · Basic Interaction (passive listen; use with QGC)

```bash
cd "20260703Python与MAVLink 2基础交互"
python basic_communication.py
python basic_communication.py --duration 60 --output my_log.txt
```

Arm and fly in QGC to see live attitude and position in the terminal.

#### Weeks 3–4 · Advanced Protocols (active control; no QGC)

```bash
cd "20260707深入MAVLink 2核心协议"
python param_ops.py              # Parameter read/write
python mission_upload.py         # Mission upload & AUTO execution
python command_control.py        # Arm → takeoff → waypoints → RTL
```

#### Weeks 4–5 · Message Signing

**Autopilot side (MAVProxy console):**

```bash
module load signing
signing setup my_sitl_mavlink2_signing_key!!!!
```

**Windows script:**

```bash
cd "20260713安全签名"
python secure_connection.py --mode all
```

#### Final Demo · Geofence Monitor

```bash
cd "20260713地理围栏实战"
python geofence_monitor.py
```

Default fence: center 100 m north of Home, radius 30 m. On entry, the vehicle hovers and flies back out to the south boundary.

---

## Scripts by Phase

### Overview

| Script                   | Phase | Protocol / Function                              | Port         |
| ------------------------ | ----- | ------------------------------------------------ | ------------ |
| `basic_communication.py` | 2–3   | Connection, HEARTBEAT, attitude/position logging | 14551 listen |
| `param_ops.py`           | 3–4   | `PARAM_*` parameter read/write                   | 14551        |
| `mission_upload.py`      | 3–4   | `MISSION_*` upload & execution                   | 14551        |
| `command_control.py`     | 3–4   | `COMMAND_LONG` arm/takeoff/RTL                   | 14551        |
| `secure_connection.py`   | 4–5   | MAVLink 2 signing (3-mode comparison)            | 14551        |
| `geofence_monitor.py`    | Final | Geofence monitoring & auto response              | 14551        |

### `basic_communication.py` (Weeks 2–3)

| Step | Function        | Key APIs                                                  |
| ---- | --------------- | --------------------------------------------------------- |
| 1    | Connect         | `mavlink_connection()` + `wait_heartbeat()`               |
| 2    | Parse heartbeat | `recv_match(type="HEARTBEAT")` → sysid / compid           |
| 3    | Request streams | `SET_MESSAGE_INTERVAL` + `REQUEST_DATA_STREAM` (4 Hz)     |
| 4    | Parse data      | `ATTITUDE` (rad→deg); `GLOBAL_POSITION_INT` (lat/lon/alt) |

Sample output:

```
[HEARTBEAT] sysid=1, compid=1, ...
[ATTITUDE]            roll=   0.11°, pitch=   0.11°, yaw=  -6.00°
[GLOBAL_POSITION_INT] lat=-35.3632622°, lon=149.1652375°, alt=584.12m
```

### Advanced Protocol Scripts (Weeks 3–4)

| Script               | Flow                                                    |
| -------------------- | ------------------------------------------------------- |
| `param_ops.py`       | Fetch 1339 params → modify demo → verify → restore      |
| `mission_upload.py`  | Upload Home + takeoff + waypoints A/B + RTL → AUTO      |
| `command_control.py` | STABILIZE arm → GUIDED → 10 m takeoff → waypoints → RTL |

**Implementation notes:**

- Mission seq=0 must be Home; AUTO mode is not armable directly — switch to STABILIZE first
- Set `RTL_ALT = --alt × 100` (cm) before RTL to avoid climbing to the default 15 m
- All scripts support `--output` for dual logging (terminal + file)

### `secure_connection.py` (Weeks 4–5)

| Mode        | Description           | Expected Result                   |
| ----------- | --------------------- | --------------------------------- |
| `signed`    | Correct 32-byte key   | 1339 params + COMMAND_ACK success |
| `unsigned`  | No signing configured | 0 params, no ACK                  |
| `wrong-key` | Wrong key             | 0 params, no ACK                  |

```bash
python secure_connection.py --mode all
python secure_connection.py --mode signed --fetch-timeout 60
python secure_connection.py --key my_sitl_mavlink2_signing_key!!!!
```

### `geofence_monitor.py` (Final Demo)

| Parameter  | Default                                   |
| ---------- | ----------------------------------------- |
| Center     | 100 m north of Home                       |
| Radius     | 30 m                                      |
| Exit point | 30 m south of center (70 m north of Home) |
| Fly target | 115 m north of Home                       |
| Altitude   | 10 m                                      |

**Auto flow:** GPS ready → arm → takeoff → fly north → enter fence → LOITER → reverse to south boundary → LOITER

```bash
python geofence_monitor.py
python geofence_monitor.py --center-north 100 --radius 30 --nav-wait 60
```

---

## Deliverables by Phase

### Week 1 · Environment Setup

- MAVLink protocol research (comparison with DDS, ROS)
- Windows (Git, Python, QGC) + Ubuntu VM (ArduPilot SITL) configuration
- QGC–SITL connection verification, MAVLink Inspector
- Troubleshooting: Git SSL, VM bridged networking, UDP ports

📁 [`20260701环境搭建和连接/`](20260701环境搭建和连接/)

### Weeks 2–3 · Basic Interaction

- `basic_communication.py`: all four core tasks completed
- Dual-port design: QGC 14550 + script 14551
- Synchronized terminal + file logging

📁 [`20260703Python与MAVLink 2基础交互/`](20260703Python与MAVLink%202基础交互/)

### Weeks 3–4 · Advanced Protocols

- Parameter protocol: 1339 params fetched, read/write verified
- Mission protocol: 5 mission items uploaded and executed in AUTO
- Command protocol: full arm → takeoff → waypoints → RTL sequence

📁 [`20260707深入MAVLink 2核心协议/`](20260707深入MAVLink%202核心协议/) · see [`README.md`](20260707深入MAVLink%202核心协议/README.md)

### Weeks 4–5 · Message Signing

- `secure_connection.py`: signing setup + three-mode comparison
- Result: accepted with correct key; rejected without key or with wrong key

📁 [`20260713安全签名/`](20260713安全签名/) · see [`README.md`](20260713安全签名/README.md)

### Final Demo · Geofence

- `geofence_monitor.py`: circular geofence + auto hover + reverse fly-out
- End-to-end SITL validation (~47 seconds)

📁 [`20260713地理围栏实战/`](20260713地理围栏实战/) · see [`README.md`](20260713地理围栏实战/README.md)

---

## Reports

| Report           | Path                                                         | Description          |
| ---------------- | ------------------------------------------------------------ | -------------------- |
| **Full project** | [`MAVLink实习项目总报告.pdf`](MAVLink实习项目总报告.pdf)     | 10 pages, all phases |
| Week 1           | [`20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex`](20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex) | Research + env setup |
| Weeks 2–3        | [`20260703Python与MAVLink 2基础交互/Python与MAVLink2基础交互汇报.tex`](20260703Python与MAVLink%202基础交互/Python与MAVLink2基础交互汇报.tex) | Basic interaction    |
| Weeks 3–4        | [`20260707深入MAVLink 2核心协议/深入MAVLink2核心协议汇报.tex`](20260707深入MAVLink%202核心协议/深入MAVLink2核心协议汇报.tex) | Advanced protocols   |
| Signing          | [`20260713安全签名/MAVLink2安全签名报告.pdf`](20260713安全签名/MAVLink2安全签名报告.pdf) | Signing verification |
| Geofence         | [`20260713地理围栏实战/地理围栏实战项目报告.pdf`](20260713地理围栏实战/地理围栏实战项目报告.pdf) | Final demo           |

---

## FAQ

### Connection & Network

| Issue                          | Cause                             | Fix                                                  |
| ------------------------------ | --------------------------------- | ---------------------------------------------------- |
| Script times out, no HEARTBEAT | SITL not running or wrong `--out` | Ensure SITL is up; use `--out=udp:<WindowsIP>:14551` |
| Still no data with `127.0.0.1` | 127.0.0.1 is the VM localhost     | Use Windows bridged IP instead                       |
| VM cannot ping Windows         | Wrong network mode                | Switch VMware to **bridged mode** and reboot VM      |
| QGC shows Disconnected         | Port/IP mismatch                  | Verify SITL sends to Windows IP:14550                |

### Flight Control

| Issue                             | Cause                   | Fix                                                |
| --------------------------------- | ----------------------- | -------------------------------------------------- |
| Cannot arm in AUTO                | AUTO is not armable     | Switch to STABILIZE first (handled in scripts)     |
| RTL climbs to ~15 m               | Default RTL_ALT=1500 cm | Set `RTL_ALT=--alt×100` before RTL                 |
| GUIDED fails: `requires position` | GPS/EKF not ready       | Wait 15–20 s after SITL start; use `--nav-wait 60` |
| PreArm: EKF3 waiting for GPS      | Same as above           | Wait for `fix_type >= 3` before arming             |
| QGC conflicts with scripts        | Both sending commands   | Disconnect QGC while running control scripts       |

### Message Signing

| Issue                           | Cause                     | Fix                                              |
| ------------------------------- | ------------------------- | ------------------------------------------------ |
| All three modes succeed         | Signing not enabled on FC | Run `signing setup KEY` in MAVProxy              |
| Script crashes after signing    | Wrong callback signature  | Use `(mav, msgId) -> bool`                       |
| Signing command has no effect   | Wrong terminal window     | Run in MAVProxy `STABILIZE>`, not the log window |
| Partial params with correct key | Timeout too short         | Use `--fetch-timeout 60`                         |

### Other

| Issue                      | Fix                                                          |
| -------------------------- | ------------------------------------------------------------ |
| Git clone SSL error        | `git config --global http.sslBackend schannel`               |
| `sim_vehicle.py` not found | Run `Tools/environment_install/install-prereqs-ubuntu.sh -y` in ArduPilot repo |

---

## Milestones

- [x] MAVLink research and environment setup
- [x] QGC–SITL connection verification
- [x] Python basic interaction (`basic_communication.py`)
- [x] Parameter read/write (`PARAM_*`)
- [x] Mission upload and execution (`MISSION_*`)
- [x] Command control (`COMMAND_LONG`)
- [x] MAVLink 2 signing verification
- [x] Geofence monitoring demo

---

## References

- [MAVLink Documentation](https://mavlink.io/en/)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [MAVLink 2 Message Signing](https://mavlink.io/en/guide/message_signing.html)
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [ArduPilot MAVLink2 Signing](https://ardupilot.org/dev/docs/common-MAVLink2-signing.html)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/)

---

## License

This repository is an internship learning project; code and docs are for educational use only. Upstream projects (ArduPilot, QGroundControl, MAVLink) are governed by their respective open-source licenses.
