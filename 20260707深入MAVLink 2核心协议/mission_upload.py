#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_upload.py - MAVLink 2 任务协议：航线上传与执行

任务要求（第四阶段 · 任务 2）：
  1. 构造至少 4 个航点的简单航线：起飞 -> 航点 A -> 航点 B -> 返航（RTL）
     （ArduPilot 需在 seq=0 额外上传 Home 参考点，共 5 条 mission item）
  2. 使用 mission_count_send() 与 mission_item_send() / mission_item_int_send() 上传
  3. 上传成功后命令飞控执行航线
  4. 在 SITL 中验证飞机按航线飞行

使用场景：
  脚本通过 UDP 14551 连接 SITL，独立完成航线上传与执行，无需 QGC。

SITL 启动示例（Ubuntu 虚拟机）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python mission_upload.py
  python mission_upload.py --upload-only       # 仅上传，不解锁执行
  python mission_upload.py --alt 15 --offset 40
  python mission_upload.py --wp-speed 500 --descent-speed 220 --land-speed 75
  python mission_upload.py --output mission_log.txt
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_ALTITUDE_M = 10.0
DEFAULT_OFFSET_M = 30.0
# ArduPilot 默认约 WPNAV_SPEED=1000、WPNAV_SPEED_DN=150、LAND_SPEED=50（单位 cm/s）
DEFAULT_WP_SPEED_CM_S = 500.0
DEFAULT_DESCENT_SPEED_CM_S = 220.0
DEFAULT_LAND_SPEED_CM_S = 75.0
MISSION_UPLOAD_TIMEOUT_SEC = 30.0
MISSION_EXEC_TIMEOUT_SEC = 300.0
COMMAND_ACK_TIMEOUT_SEC = 5.0
PARAM_IO_TIMEOUT_SEC = 5.0
MODE_CHANGE_TIMEOUT_SEC = 15.0

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


@dataclass
class MissionWaypoint:
    """单个任务航点。"""

    seq: int
    command: int
    lat: float
    lon: float
    alt: float
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    label: str = ""

    @property
    def command_name(self) -> str:
        try:
            return mavutil.mavlink.enums["MAV_CMD"][self.command].name
        except KeyError:
            return str(self.command)


class OutputLogger:
    """日志记录器：stderr 显示 + 文件写入。"""

    def __init__(self, filepath: Path, title: str):
        self.filepath = filepath.resolve()
        self.title = title
        self._header_written = False
        self._file = self.filepath.open("w", encoding="utf-8", newline="\n")
        self._write_header()

    def _write_header(self):
        if self._header_written:
            return
        lines = [
            self.title,
            f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"监听端口: UDP {DEFAULT_LISTEN_PORT}",
            "=" * 72,
        ]
        self._file.write("\n".join(lines) + "\n")
        self._file.flush()
        self._header_written = True

    def log(self, message: str = ""):
        print(message, file=sys.stderr)
        self._file.write(message + "\n")
        self._file.flush()

    def close(self):
        footer_lines = [
            "=" * 72,
            f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        ]
        self._file.write("\n".join(footer_lines) + "\n")
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def parse_args():
    parser = argparse.ArgumentParser(
        description="MAVLink 2 任务协议：构造、上传并执行简单航线"
    )
    parser.add_argument(
        "--connection",
        default=f"udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}",
        help=f"MAVLink 连接字符串，默认 udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="等待心跳超时（秒），默认 10",
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=DEFAULT_ALTITUDE_M,
        help=f"任务高度（米，相对 Home），默认 {DEFAULT_ALTITUDE_M}",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=DEFAULT_OFFSET_M,
        help=f"航点相对 Home 的偏移距离（米），默认 {DEFAULT_OFFSET_M}",
    )
    parser.add_argument(
        "--upload-only",
        action="store_true",
        help="仅上传航线，不解锁、不执行",
    )
    parser.add_argument(
        "--exec-timeout",
        type=float,
        default=MISSION_EXEC_TIMEOUT_SEC,
        help=f"任务执行监控超时（秒），默认 {MISSION_EXEC_TIMEOUT_SEC:.0f}",
    )
    parser.add_argument(
        "--wp-speed",
        type=float,
        default=DEFAULT_WP_SPEED_CM_S,
        help=f"航线水平速度 WPNAV_SPEED（cm/s），默认 {DEFAULT_WP_SPEED_CM_S:g}（略低于飞控默认 1000）",
    )
    parser.add_argument(
        "--descent-speed",
        type=float,
        default=DEFAULT_DESCENT_SPEED_CM_S,
        help=f"下降/返航垂直速度 WPNAV_SPEED_DN（cm/s），默认 {DEFAULT_DESCENT_SPEED_CM_S:g}（略高于默认 150）",
    )
    parser.add_argument(
        "--land-speed",
        type=float,
        default=DEFAULT_LAND_SPEED_CM_S,
        help=f"最终降落速度 LAND_SPEED（cm/s），默认 {DEFAULT_LAND_SPEED_CM_S:g}（略高于默认 50）",
    )
    parser.add_argument(
        "--keep-speeds",
        action="store_true",
        help="任务结束后保留修改后的速度/返航高度参数（默认会恢复原值）",
    )
    parser.add_argument(
        "--output",
        default="",
        help="日志文件路径；默认自动生成 mission_output_时间戳.txt",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"mission_output_{stamp}.txt"


def connect(logger: OutputLogger, connection_string: str, timeout: float):
    logger.log(f"[连接] 正在监听: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)
    logger.log("[连接] 等待 HEARTBEAT（请确认 SITL 已启动且 --out 包含 14551）...")
    master.wait_heartbeat(timeout=timeout)
    logger.log(
        f"[连接] 已收到心跳 | target_system={master.target_system}, "
        f"target_component={master.target_component}"
    )
    return master


def offset_position(lat: float, lon: float, north_m: float, east_m: float):
    """以米为单位向北/向东偏移经纬度。"""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def get_home_position(logger: OutputLogger, master, timeout: float = 10.0):
    """从 GLOBAL_POSITION_INT 或 HOME_POSITION 获取 Home 附近坐标。"""
    logger.log("[位置] 等待 GLOBAL_POSITION_INT / HOME_POSITION...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HOME_POSITION"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue
        if msg.get_type() == "HOME_POSITION":
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            logger.log(f"[位置] HOME_POSITION | lat={lat:.7f}°, lon={lon:.7f}°")
            return lat, lon
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        logger.log(f"[位置] GLOBAL_POSITION_INT | lat={lat:.7f}°, lon={lon:.7f}°")
        return lat, lon

    lat, lon = -35.363261, 149.165230
    logger.log(f"[位置] 超时，使用 SITL 默认 Home | lat={lat:.7f}°, lon={lon:.7f}°")
    return lat, lon


def build_mission_waypoints(
    home_lat: float,
    home_lon: float,
    altitude_m: float,
    offset_m: float,
) -> List[MissionWaypoint]:
    """
    构造 ArduPilot Copter 兼容任务。

    ArduPilot 要求 seq=0 为 Home 参考点（NAV_WAYPOINT），
    实际飞行从 seq=1 的 TAKEOFF 开始：起飞 -> A -> B -> RTL。
    """
    lat_a, lon_a = offset_position(home_lat, home_lon, offset_m, 0.0)
    lat_b, lon_b = offset_position(home_lat, home_lon, offset_m, offset_m)
    mav = mavutil.mavlink

    return [
        MissionWaypoint(
            0,
            mav.MAV_CMD_NAV_WAYPOINT,
            home_lat,
            home_lon,
            0.0,
            label="Home",
        ),
        MissionWaypoint(
            1,
            mav.MAV_CMD_NAV_TAKEOFF,
            0.0,
            0.0,
            altitude_m,
            label="起飞",
        ),
        MissionWaypoint(
            2,
            mav.MAV_CMD_NAV_WAYPOINT,
            lat_a,
            lon_a,
            altitude_m,
            label="航点 A",
        ),
        MissionWaypoint(
            3,
            mav.MAV_CMD_NAV_WAYPOINT,
            lat_b,
            lon_b,
            altitude_m,
            label="航点 B",
        ),
        MissionWaypoint(
            4,
            mav.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0.0,
            0.0,
            0.0,
            label="返航 RTL",
        ),
    ]


def log_mission_plan(logger: OutputLogger, waypoints: List[MissionWaypoint]):
    logger.log("[航线] 任务航点规划：")
    logger.log("-" * 72)
    for wp in waypoints:
        if wp.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            coord = "（飞控自动返航）"
        elif wp.label == "Home":
            coord = f"lat={wp.lat:.7f}°, lon={wp.lon:.7f}°（Home 参考点）"
        elif wp.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            coord = f"alt={wp.alt:.1f}m"
        else:
            coord = f"lat={wp.lat:.7f}°, lon={wp.lon:.7f}°, alt={wp.alt:.1f}m"
        logger.log(
            f"  seq={wp.seq}  {wp.label:<8}  {wp.command_name:<24}  {coord}"
        )
    logger.log("-" * 72)


def send_mission_item(logger: OutputLogger, master, wp: MissionWaypoint, use_int: bool):
    """按请求类型发送 MISSION_ITEM_INT 或 MISSION_ITEM。"""
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    current = 0
    autocontinue = 1

    if use_int:
        logger.log(
            f"[上传] mission_item_int_send seq={wp.seq} "
            f"({wp.label}, {wp.command_name})"
        )
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            wp.seq,
            frame,
            wp.command,
            current,
            autocontinue,
            wp.param1,
            wp.param2,
            wp.param3,
            wp.param4,
            int(wp.lat * 1e7),
            int(wp.lon * 1e7),
            wp.alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    else:
        logger.log(
            f"[上传] mission_item_send seq={wp.seq} "
            f"({wp.label}, {wp.command_name})"
        )
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            wp.seq,
            frame,
            wp.command,
            current,
            autocontinue,
            wp.param1,
            wp.param2,
            wp.param3,
            wp.param4,
            wp.lat,
            wp.lon,
            wp.alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )


def wait_mission_ack(logger: OutputLogger, master, timeout: float, action: str):
    msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError(f"{action} 未收到 MISSION_ACK（{timeout}s）")
    logger.log(f"[任务] {action} | MISSION_ACK type={msg.type}")
    return msg


def upload_mission(
    logger: OutputLogger,
    master,
    waypoints: List[MissionWaypoint],
    timeout: float = MISSION_UPLOAD_TIMEOUT_SEC,
):
    """清空、上传航线并等待 MISSION_ACK。"""
    logger.log("[任务] mission_clear_all_send() 清空现有航线...")
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    try:
        wait_mission_ack(logger, master, 5.0, "清空航线确认")
    except TimeoutError:
        logger.log("[任务] 未收到清空确认（部分固件不回复，继续上传）")

    count = len(waypoints)
    logger.log(f"[任务] mission_count_send() 航点数量 = {count}")
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )

    sent = set()
    deadline = time.time() + timeout
    while len(sent) < count:
        if time.time() > deadline:
            raise TimeoutError(f"航线上传超时，已发送 {len(sent)}/{count} 个航点")

        req = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
            blocking=True,
            timeout=2.0,
        )
        if req is None:
            continue

        seq = req.seq
        if seq < 0 or seq >= count:
            logger.log(f"[警告] 收到非法 seq={seq}，忽略")
            continue

        use_int = req.get_type() == "MISSION_REQUEST_INT"
        send_mission_item(logger, master, waypoints[seq], use_int=use_int)
        sent.add(seq)

    ack = wait_mission_ack(logger, master, 10.0, "上传完成确认")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"航线上传被拒绝，MISSION_ACK type={ack.type}")
    logger.log(f"[任务] 航线上传成功，共 {count} 个航点")


def wait_command_ack(master, command: int, timeout: float = COMMAND_ACK_TIMEOUT_SEC):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.command == command:
            return msg
    return None


def param_id_to_str(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode("utf-8", errors="replace").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def param_id_to_bytes(name: str) -> bytes:
    return name.encode("utf-8")[:16]


def read_param(logger: OutputLogger, master, param_name: str):
    logger.log(f"[参数] param_request_read -> {param_name}")
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_id_to_bytes(param_name),
        -1,
    )
    deadline = time.time() + PARAM_IO_TIMEOUT_SEC
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if param_id_to_str(msg.param_id) == param_name:
            logger.log(
                f"[参数] {param_name} = {msg.param_value:g} (type={msg.param_type})"
            )
            return msg
    raise TimeoutError(f"读取参数 {param_name} 超时")


def set_param_value(logger: OutputLogger, master, param_name: str, value: float, param_type: int):
    logger.log(f"[参数] param_set -> {param_name} = {value:g}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_to_bytes(param_name),
        float(value),
        param_type,
    )
    deadline = time.time() + PARAM_IO_TIMEOUT_SEC
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if param_id_to_str(msg.param_id) != param_name:
            continue
        logger.log(f"[参数] 确认 {param_name} = {msg.param_value:g}")
        return msg
    raise TimeoutError(f"写入参数 {param_name} 未收到确认")


def apply_mission_flight_params(
    logger: OutputLogger,
    master,
    wp_speed: float,
    descent_speed: float,
    land_speed: float,
    rtl_alt_m: float,
):
    """调整任务速度及 RTL 返航高度，返回原值列表供恢复。"""
    rtl_alt_cm = rtl_alt_m * 100.0
    targets = {
        "WPNAV_SPEED": wp_speed,
        "WPNAV_SPEED_DN": descent_speed,
        "LAND_SPEED": land_speed,
        "RTL_ALT": rtl_alt_cm,
    }
    logger.log("[飞行参数] 调整速度与返航高度：")
    logger.log(
        f"  WPNAV_SPEED={wp_speed:g} cm/s ({wp_speed/100:.1f} m/s) 水平飞行"
    )
    logger.log(
        f"  WPNAV_SPEED_DN={descent_speed:g} cm/s ({descent_speed/100:.1f} m/s) 下降/返航"
    )
    logger.log(
        f"  LAND_SPEED={land_speed:g} cm/s ({land_speed/100:.1f} m/s) 最终降落"
    )
    logger.log(
        f"  RTL_ALT={rtl_alt_cm:g} cm ({rtl_alt_m:.1f} m) 返航高度（与任务高度一致）"
    )

    originals = []
    for name, target in targets.items():
        msg = read_param(logger, master, name)
        originals.append((name, msg.param_value, msg.param_type))
        if abs(msg.param_value - target) <= 1e-3:
            logger.log(f"[飞行参数] {name} 已是目标值，跳过写入")
            continue
        set_param_value(logger, master, name, target, msg.param_type)
    return originals


def restore_mission_flight_params(logger: OutputLogger, master, originals):
    logger.log("[飞行参数] 恢复速度与返航高度原值...")
    for name, value, param_type in originals:
        set_param_value(logger, master, name, value, param_type)
    logger.log("[飞行参数] 参数已恢复")


def arm_vehicle(logger: OutputLogger, master, timeout: float = 30.0) -> bool:
    if master.motors_armed():
        logger.log("[执行] 飞行器已解锁")
        return True

    logger.log("[执行] command_long_send -> MAV_CMD_COMPONENT_ARM_DISARM (解锁)")
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0,
        )
        ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack is not None:
            logger.log(f"[执行] ARM COMMAND_ACK | result={mav_result_name(ack.result)}")

        log_statustext(logger, master)
        refresh_vehicle_state(master, timeout=2.0)
        if master.motors_armed():
            logger.log("[执行] 解锁成功")
            return True

        time.sleep(1.0)

    logger.log("[执行] 解锁失败，请检查 SITL 预解锁条件（GPS、传感器等）")
    return False


def mav_result_name(result: int) -> str:
    return MAV_RESULT_NAMES.get(result, str(result))


def refresh_vehicle_state(master, timeout: float = 2.0):
    """等待并刷新来自目标飞控的 HEARTBEAT 状态。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_srcSystem() == master.target_system:
            return msg
    return None


def resolve_mode_number(master, mode_name: str) -> Optional[int]:
    mapping = master.mode_mapping()
    if mapping and mode_name in mapping:
        return mapping[mode_name]
    fallback = {
        "STABILIZE": 0,
        "AUTO": 3,
        "GUIDED": 4,
        "RTL": 6,
    }
    return fallback.get(mode_name.upper())


def wait_for_flight_mode(
    logger: OutputLogger,
    master,
    mode_name: str,
    timeout: float,
) -> bool:
    target = mode_name.upper()
    deadline = time.time() + timeout
    while time.time() < deadline:
        refresh_vehicle_state(master, timeout=1.0)
        mode = getattr(master, "flightmode", None) or "UNKNOWN"
        if target in mode.upper():
            logger.log(f"[执行] 当前模式: {mode}")
            return True
        time.sleep(0.5)
    logger.log(
        f"[执行] 切换 {mode_name} 超时，当前模式: "
        f"{getattr(master, 'flightmode', 'UNKNOWN')}"
    )
    return False


def log_statustext(logger: OutputLogger, master, duration: float = 0.3):
    """读取飞控 STATUSTEXT，便于定位解锁/模式失败原因。"""
    deadline = time.time() + duration
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            break
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        text = text.rstrip("\x00").strip()
        if text:
            logger.log(f"[飞控] {text}")


def set_flight_mode(
    logger: OutputLogger,
    master,
    mode_name: str,
    timeout: float = MODE_CHANGE_TIMEOUT_SEC,
) -> bool:
    mode_number = resolve_mode_number(master, mode_name)
    if mode_number is None:
        logger.log(f"[执行] 未知模式 {mode_name}")
        return False

    logger.log(f"[执行] 切换飞行模式 -> {mode_name}（mode={mode_number}）")
    if mode_name.upper() in (getattr(master, "flightmode", "") or "").upper():
        logger.log(f"[执行] 已在 {mode_name} 模式 | mode={master.flightmode}")
        return True

    logger.log(f"[执行] set_mode_apm('{mode_name}')...")
    master.set_mode_apm(mode_name)
    if wait_for_flight_mode(logger, master, mode_name, timeout / 3):
        return True

    logger.log("[执行] 尝试 set_mode_send()...")
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if master.motors_armed():
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    master.mav.set_mode_send(master.target_system, base_mode, mode_number)
    if wait_for_flight_mode(logger, master, mode_name, timeout / 3):
        return True

    logger.log("[执行] 尝试 command_long_send -> DO_SET_MODE...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_number,
        0, 0, 0, 0, 0,
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if ack is not None:
        logger.log(
            f"[执行] DO_SET_MODE COMMAND_ACK | result={mav_result_name(ack.result)}"
        )
    log_statustext(logger, master)

    return wait_for_flight_mode(logger, master, mode_name, timeout / 3)


def set_auto_mode(logger: OutputLogger, master, timeout: float = MODE_CHANGE_TIMEOUT_SEC) -> bool:
    return set_flight_mode(logger, master, "AUTO", timeout)


def start_mission(logger: OutputLogger, master) -> bool:
    """发送 MISSION_START，确保 AUTO 模式下任务开始。"""
    logger.log("[执行] command_long_send -> MAV_CMD_MISSION_START")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_MISSION_START)
    if ack is None:
        logger.log("[执行] 未收到 MISSION_START 确认（部分版本会自动开始，继续监控）")
        return True
    logger.log(f"[执行] MISSION_START COMMAND_ACK | result={mav_result_name(ack.result)}")
    return ack.result in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )


def execute_mission(logger: OutputLogger, master) -> bool:
    """
    ArduPilot 在 AUTO 模式下通常不能直接解锁（QGC: AUTO mode not armable）。
    正确顺序：STABILIZE 解锁 -> 切 AUTO -> MISSION_START。
    """
    if not set_flight_mode(logger, master, "STABILIZE"):
        return False
    if not arm_vehicle(logger, master):
        logger.log(
            "[提示] 解锁失败常见原因：\n"
            "  1. 当前处于 AUTO 模式（不可解锁）\n"
            "  2. QGC 与脚本同时控制，请断开 QGC 或关闭其自动连接\n"
            "  3. GPS/PreArm 检查未通过，查看上方 [飞控] 提示"
        )
        return False
    if not set_auto_mode(logger, master):
        return False
    start_mission(logger, master)
    logger.log("[执行] 任务已开始，飞控将按航线：起飞 -> A -> B -> RTL")
    return True


def monitor_mission(
    logger: OutputLogger,
    master,
    waypoint_count: int,
    timeout: float,
):
    """监控任务进度，记录航点切换与位置变化。"""
    logger.log(f"[监控] 开始监控任务执行（超时 {timeout:.0f}s）")
    logger.log("-" * 72)

    start = time.time()
    last_seq = -1
    last_pos_log = 0.0
    reached_rtl = False

    while time.time() - start < timeout:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "MISSION_CURRENT":
            if msg.seq != last_seq:
                logger.log(f"[进度] 当前航点 seq={msg.seq}/{waypoint_count - 1}")
                last_seq = msg.seq
                if msg.seq >= waypoint_count - 1:
                    reached_rtl = True

        elif msg_type == "MISSION_ITEM_REACHED":
            logger.log(f"[进度] 已到达航点 seq={msg.seq}")

        elif msg_type == "GLOBAL_POSITION_INT":
            now = time.time()
            if now - last_pos_log >= 3.0:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                rel_alt = msg.relative_alt / 1000.0
                logger.log(
                    f"[位置] lat={lat:.7f}°, lon={lon:.7f}°, "
                    f"relative_alt={rel_alt:.2f}m"
                )
                last_pos_log = now

        elif msg_type == "HEARTBEAT":
            if msg.get_srcSystem() != master.target_system:
                continue
            if reached_rtl and not master.motors_armed():
                mode = mavutil.mode_string_v10(msg)
                logger.log(f"[进度] 返航完成，已上锁 | mode={mode}")
                return True

    logger.log("[监控] 监控超时，请手动在 SITL/QGC 中查看飞机状态")
    return False


def main():
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    with OutputLogger(output_path, "MAVLink 2 任务协议上传与执行记录") as logger:
        logger.log(f"[输出] 日志文件: {output_path}")

        try:
            master = connect(logger, args.connection, args.timeout)
        except Exception as exc:
            logger.log(f"[错误] 连接失败: {exc}")
            logger.log(
                "[提示] 请确认：\n"
                "  1. 虚拟机中 SITL 正在运行\n"
                "  2. 启动命令包含 --out=udp:<WindowsIP>:14551\n"
                "  3. 本脚本主动控制飞控，请勿与 QGC 同时发送指令"
            )
            return 1

        home_lat, home_lon = get_home_position(logger, master)
        waypoints = build_mission_waypoints(
            home_lat, home_lon, args.alt, args.offset
        )
        log_mission_plan(logger, waypoints)

        try:
            upload_mission(logger, master, waypoints)
        except Exception as exc:
            logger.log(f"[错误] 航线上传失败: {exc}")
            return 1

        if args.upload_only:
            logger.log("[结束] --upload-only 模式，已上传航线但未执行")
            logger.log(f"[输出] 记录已保存至: {output_path}")
            return 0

        speed_originals = None
        try:
            speed_originals = apply_mission_flight_params(
                logger,
                master,
                args.wp_speed,
                args.descent_speed,
                args.land_speed,
                args.alt,
            )

            if not execute_mission(logger, master):
                logger.log("[错误] 任务执行启动失败")
                return 1

            monitor_mission(
                logger,
                master,
                len(waypoints),
                args.exec_timeout,
            )
        finally:
            if speed_originals and not args.keep_speeds:
                restore_mission_flight_params(logger, master, speed_originals)

        logger.log(f"[输出] 记录已保存至: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
