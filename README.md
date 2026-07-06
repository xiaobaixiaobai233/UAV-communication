# UAV Communication · MAVLink2 Project

A learning and experimentation repository for **MAVLink 2**–based drone communication. It documents Weeks 1–3 of an internship: environment setup and basic Python interaction with an ArduPilot SITL simulator. The project uses a **Windows host + Ubuntu VM (SITL) + QGroundControl** architecture, with `pymavlink` for communication and data parsing.

---

## Progress

| Phase      | Task                                                         | Status    |
| ---------- | ------------------------------------------------------------ | --------- |
| Week 1     | MAVLink research + dev environment setup + QGC–SITL connection verification | ✅ Done    |
| Weeks 2–3  | Python & MAVLink 2 basic interaction (`basic_communication.py`) | ✅ Done    |
| Weeks 3–4  | Parameter read/write, mission upload, command control        | 🔲 Planned |
| Weeks 4–5  | MAVLink 2 signing enablement & verification                  | 🔲 Planned |
| Final goal | Geofence monitoring + auto hover / return script             | 🔲 Planned |

---

## Repository Structure

```
无人机通信/
├── README.md
├── 任务.txt                                     # Internship task schedule
├── 环境搭建操作步骤.txt                          # Environment setup checklist
│
├── 20260701环境搭建和连接/                       # Week 1 deliverables
│   ├── MAVLink环境搭建与连接验证报告.tex         # Protocol research + env config + verification report
│   └── screenshots/                             # Verification screenshots
│
└── 20260703Python与MAVLink 2基础交互/            # Weeks 2–3 deliverables
    ├── basic_communication.py                   # Core script: MAVLink connection & data parsing
    ├── requirements.txt                         # Python dependencies
    ├── mavlink_output_20260703_164931.txt       # Sample run log
    └── Python与MAVLink2基础交互汇报.tex          # Script design & experiment report
```

> External tools (QGroundControl, ArduPilot source, etc.) are not included due to size. Install them locally following the documentation.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Ubuntu VM (VMware)                                         │
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
│  │  QGroundControl      │   │  basic_communication.py     │ │
│  │  UDP 14550 (control) │   │  UDP 14551 (passive listen) │ │
│  └──────────────────────┘   └─────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

**Design note:** QGC handles **control** (arm, takeoff, mode changes). The Python script **passively listens** to vehicle state and writes logs. The dual-port setup prevents QGC and the script from competing for the same UDP port.

---

## Requirements

| Component       | Version / Notes                                    |
| --------------- | -------------------------------------------------- |
| OS              | Windows 10 + Ubuntu VM (VMware bridged networking) |
| Python          | 3.10+                                              |
| Ground station  | QGroundControl 4.2+ (MAVLink 2 support)            |
| Flight sim      | ArduPilot SITL (ArduCopter, Copter-4.4 branch)     |
| Python library  | `pymavlink >= 2.4.41`                              |
| Version control | Git 2.47+                                          |

For detailed setup steps, see [`环境搭建操作步骤.txt`](环境搭建操作步骤.txt) and [`20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex`](20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex).

---

## Quick Start

### 1. Start SITL (Ubuntu VM)

```bash
cd ~/ardupilot/ArduCopter
python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \
    --out=udp:<WindowsHostIP>:14550 \
    --out=udp:<WindowsHostIP>:14551
```

- `14550` — QGroundControl
- `14551` — Python script

> Replace `<WindowsHostIP>` with your Windows LAN IP (use `ipconfig` in bridged mode). The first run compiles SITL and may take a while; wait until you see the `STABILIZE>` prompt.

### 2. Connect QGC (Windows)

1. Launch QGroundControl; it should auto-connect on UDP 14550
2. After connection, the simulated vehicle appears on the map
3. Use **Analyze → MAVLink Inspector** to inspect `HEARTBEAT`, `GPS_RAW_INT`, etc.

### 3. Run the Python Script (Windows)

```bash
cd "20260703Python与MAVLink 2基础交互"
pip install -r requirements.txt
python basic_communication.py
```

Optional arguments:

```bash
python basic_communication.py --duration 60          # Exit after 60 seconds
python basic_communication.py --output my_log.txt    # Custom log file path
python basic_communication.py --timeout 15           # Heartbeat wait timeout (seconds)
```

Arm, take off, or fly a mission in QGC to see live attitude and position data in the terminal. Press `Ctrl+C` to stop.

---

## Core Script

[`basic_communication.py`](20260703Python与MAVLink%202基础交互/basic_communication.py) implements four required tasks:

| Step | Function                            | Key APIs                                                     |
| ---- | ----------------------------------- | ------------------------------------------------------------ |
| 1    | Establish MAVLink connection        | `mavutil.mavlink_connection()` + `wait_heartbeat()`          |
| 2    | Receive & parse HEARTBEAT           | `recv_match(type="HEARTBEAT")`, print `sysid` / `compid`     |
| 3    | Request attitude & position streams | `SET_MESSAGE_INTERVAL` + `REQUEST_DATA_STREAM` (4 Hz)        |
| 4    | Parse & output flight data          | `ATTITUDE` → Euler angles; `GLOBAL_POSITION_INT` → lat/lon/altitude |

### Sample Output

```
[连接] 已收到心跳 | target_system=1, target_component=0
[HEARTBEAT] sysid=1, compid=1, type=2, autopilot=3, base_mode=217, custom_mode=3, system_status=4
[请求] SET_MESSAGE_INTERVAL -> ATTITUDE @ 4Hz
[请求] SET_MESSAGE_INTERVAL -> GLOBAL_POSITION_INT @ 4Hz
[ATTITUDE]            roll=   0.11°, pitch=   0.11°, yaw=  -6.00°
[GLOBAL_POSITION_INT] lat=-35.3632622°, lon=149.1652375°, alt=584.12m, relative_alt=0.07m
```

> Log messages are in Chinese; field names and values are self-explanatory.

The `OutputLogger` class mirrors all output to both the terminal and a timestamped `mavlink_output_YYYYMMDD_HHMMSS.txt` file.

### Data Fields

| Message               | Output Fields    | Unit Conversion                         |
| --------------------- | ---------------- | --------------------------------------- |
| `ATTITUDE`            | roll, pitch, yaw | radians → degrees (°)                   |
| `GLOBAL_POSITION_INT` | lat, lon         | integer × 10⁷ → degrees (°)             |
| `GLOBAL_POSITION_INT` | alt              | millimeters → meters (m, AMSL)          |
| `GLOBAL_POSITION_INT` | relative_alt     | millimeters → meters (m, above takeoff) |

---

## Week 1 Deliverables

- **MAVLink research:** protocol overview, comparison with DDS/ROS, system architecture and roles
- **Environment setup:** Windows (Git, Python, QGC) + Ubuntu VM (ArduPilot SITL)
- **Connection verification:** QGC–SITL link, MAVLink Inspector, arm/takeoff command feedback
- **Troubleshooting notes:** Git SSL, Qt Kit configuration, VM bridged networking, UDP port conflicts

See [`20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex`](20260701环境搭建和连接/MAVLink环境搭建与连接验证报告.tex).

---

## Weeks 2–3 Deliverables

- Implemented `basic_communication.py`: MAVLink 2 connection, heartbeat parsing, stream subscription, attitude/position parsing
- Dual-port design: QGC (14550) + Python (14551)
- Synchronized terminal + file logging
- End-to-end validation with ArduPilot SITL (Euler angles, lat/lon, relative altitude)

See [`20260703Python与MAVLink 2基础交互/Python与MAVLink2基础交互汇报.tex`](20260703Python与MAVLink%202基础交互/Python与MAVLink2基础交互汇报.tex).

---

## FAQ

| Issue                               | Fix                                                          |
| ----------------------------------- | ------------------------------------------------------------ |
| Script times out, no HEARTBEAT      | Ensure SITL is running and `--out` includes your Windows IP and port 14551 |
| QGC shows Disconnected              | Verify SITL sends to Windows IP:14550; disable QGC NMEA on port 14550 |
| VM cannot ping Windows host         | Switch VMware to **bridged mode** and reboot the VM          |
| `sim_vehicle.py: command not found` | Run `Tools/environment_install/install-prereqs-ubuntu.sh -y` in the ArduPilot repo, then restart the terminal |
| Git clone SSL error                 | Run `git config --global http.sslBackend schannel`           |

---

## Roadmap

- [ ] Parameter read/write (`PARAM_REQUEST_READ` / `PARAM_SET`)
- [ ] Mission waypoint upload and execution
- [ ] Command control (arm, takeoff, mode switch, etc.)
- [ ] MAVLink 2 signing enablement and verification
- [ ] Geofence monitoring with auto hover / return script

---

## References

- [MAVLink Documentation](https://mavlink.io/en/)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/)

---

## License

This repository is an internship learning project; code and docs are for educational use only. Upstream projects (ArduPilot, QGroundControl, MAVLink) are governed by their respective open-source licenses.
