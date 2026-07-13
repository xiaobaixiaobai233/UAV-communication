#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geofence_monitor.py - 地理围栏监测：进入围栏自动悬停并反向飞出至边界

围栏设定（默认）：
  - 圆心：Home 点正北 100m
  - 半径：30m
  - 触发：进入圆内 → LOITER 悬停 → 反向（向南）飞出 → 停在南边界悬停

测试流程：
  Home 解锁起飞 → GUIDED 向北飞 → 进入围栏触发 → 反向至南边界（Home 北 70m）悬停

SITL 启动（Ubuntu VM）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python geofence_monitor.py
  python geofence_monitor.py --alt 10 --center-north 100 --radius 30
  python geofence_monitor.py --output geofence_log.txt
"""

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_ALT_M = 10.0
DEFAULT_CENTER_NORTH_M = 100.0
DEFAULT_FENCE_RADIUS_M = 30.0
DEFAULT_FLY_TARGET_NORTH_M = 115.0
DEFAULT_HOVER_IN_FENCE_SEC = 3.0
DEFAULT_HOVER_AT_EDGE_SEC = 8.0
DEFAULT_NAV_READY_TIMEOUT_SEC = 45.0
DEFAULT_SITL_WARMUP_SEC = 8.0

COMMAND_ACK_TIMEOUT_SEC = 8.0
MODE_CHANGE_TIMEOUT_SEC = 15.0
TAKEOFF_TIMEOUT_SEC = 60.0
GOTO_TIMEOUT_SEC = 120.0
FENCE_MONITOR_TIMEOUT_SEC = 180.0
ALTITUDE_TOLERANCE_M = 0.8
POSITION_TOLERANCE_M = 3.0
ARM_MAX_ATTEMPTS = 5
NON_ARMABLE_MODES = ("AUTO", "RTL", "LAND", "LOITER", "BRAKE", "SMART_RTL", "GUIDED")

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


class OutputLogger:
    def __init__(self, filepath: Path):
        self.filepath = filepath.resolve()
        self._file = self.filepath.open("w", encoding="utf-8", newline="\n")
        self._write_header()

    def _write_header(self):
        lines = [
            "地理围栏监测 · 进入围栏反向飞出",
            f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"监听端口: UDP {DEFAULT_LISTEN_PORT}",
            "=" * 72,
        ]
        self._file.write("\n".join(lines) + "\n")
        self._file.flush()

    def log(self, message: str = ""):
        print(message, file=sys.stderr)
        self._file.write(message + "\n")
        self._file.flush()

    def close(self):
        self._file.write(
            "\n".join(["=" * 72, f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", ""])
        )
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def parse_args():
    parser = argparse.ArgumentParser(description="地理围栏监测：进入圆区自动悬停并反向飞出")
    parser.add_argument(
        "--connection",
        default=f"udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}",
        help=f"MAVLink 连接，默认 udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}",
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="心跳超时（秒）")
    parser.add_argument("--alt", type=float, default=DEFAULT_ALT_M, help="飞行高度（米）")
    parser.add_argument(
        "--center-north",
        type=float,
        default=DEFAULT_CENTER_NORTH_M,
        help=f"围栏圆心相对 Home 北向偏移（米），默认 {DEFAULT_CENTER_NORTH_M}",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=DEFAULT_FENCE_RADIUS_M,
        help=f"围栏半径（米），默认 {DEFAULT_FENCE_RADIUS_M}",
    )
    parser.add_argument(
        "--fly-target-north",
        type=float,
        default=DEFAULT_FLY_TARGET_NORTH_M,
        help=(
            f"测试时向北飞行的目标点（相对 Home 北向米数），"
            f"默认 {DEFAULT_FLY_TARGET_NORTH_M}（位于围栏内）"
        ),
    )
    parser.add_argument(
        "--hover-in-fence",
        type=float,
        default=DEFAULT_HOVER_IN_FENCE_SEC,
        help="进入围栏后悬停秒数",
    )
    parser.add_argument(
        "--hover-at-edge",
        type=float,
        default=DEFAULT_HOVER_AT_EDGE_SEC,
        help="到达南边界后悬停秒数",
    )
    parser.add_argument(
        "--nav-wait",
        type=float,
        default=DEFAULT_NAV_READY_TIMEOUT_SEC,
        help=f"等待 GPS/EKF 就绪超时（秒），默认 {DEFAULT_NAV_READY_TIMEOUT_SEC:.0f}",
    )
    parser.add_argument("--output", default="", help="日志路径")
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"geofence_output_{stamp}.txt"


def mav_result_name(result: int) -> str:
    return MAV_RESULT_NAMES.get(result, str(result))


def ack_ok(result: int) -> bool:
    return result in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )


def offset_position(lat: float, lon: float, north_m: float, east_m: float) -> Tuple[float, float]:
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def horizontal_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def in_geofence(
    lat: float, lon: float, center_lat: float, center_lon: float, radius_m: float
) -> bool:
    return horizontal_distance_m(lat, lon, center_lat, center_lon) <= radius_m


def connect(logger: OutputLogger, connection_string: str, timeout: float):
    logger.log(f"[连接] 正在监听: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)
    logger.log("[连接] 等待 HEARTBEAT...")
    master.wait_heartbeat(timeout=timeout)
    logger.log(
        f"[连接] 已收到心跳 | target_system={master.target_system}, "
        f"target_component={master.target_component}"
    )
    return master


def request_data_streams(logger: OutputLogger, master):
    """请求位置与 GPS 数据流，便于 EKF 收敛与围栏监测。"""
    target_system = master.target_system
    target_component = master.target_component
    streams = [
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT"),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, "GPS_RAW_INT"),
    ]
    for msg_id, name in streams:
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            250_000,
            0,
            0,
            0,
            0,
            0,
        )
        logger.log(f"[数据流] 已请求 {name} @ 4Hz")


def wait_for_navigation_ready(
    logger: OutputLogger, master, timeout: float, warmup_sec: float
) -> bool:
    """
    等待 SITL GPS/EKF 就绪后再解锁或切 GUIDED。

    GUIDED 需要有效全球位置；刚连上 SITL 时常见：
      PreArm: EKF3 waiting for GPS config data
      Mode change to GUIDED failed: requires position
    """
    logger.log(
        f"[准备] 等待 GPS/EKF 就绪（预热 {warmup_sec:.0f}s，超时 {timeout:.0f}s）..."
    )
    logger.log("[准备] SITL 刚启动时请稍候，直到飞控完成定位初始化")
    time.sleep(warmup_sec)

    deadline = time.time() + timeout
    last_log = 0.0
    gps_fix_ok = False
    position_ok = False

    while time.time() < deadline:
        gps_msg = master.recv_match(type="GPS_RAW_INT", blocking=False)
        if gps_msg is not None and gps_msg.fix_type >= 3:
            gps_fix_ok = True

        pos_msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if pos_msg is not None:
            lat = pos_msg.lat / 1e7
            lon = pos_msg.lon / 1e7
            if abs(lat) > 1e-5 and abs(lon) > 1e-5:
                position_ok = True

        log_statustext(logger, master, duration=0.2)

        if time.time() - last_log >= 2.0:
            fix_text = gps_msg.fix_type if gps_msg is not None else "N/A"
            logger.log(
                f"[准备] GPS fix>={3}: {'是' if gps_fix_ok else '否'} "
                f"(fix_type={fix_text}), "
                f"全球位置: {'是' if position_ok else '否'}"
            )
            last_log = time.time()

        if gps_fix_ok and position_ok:
            logger.log("[准备] GPS/EKF 已就绪，可解锁并切换 GUIDED")
            time.sleep(1.0)
            return True

        time.sleep(0.2)

    logger.log(
        "[错误] GPS/EKF 等待超时。请确认：\n"
        "  1. SITL 已完全启动（MAVProxy 出现 STABILIZE> 后再等 10~20s）\n"
        "  2. 仿真正常（地图上有飞机、GPS 状态正常）\n"
        "  3. 增大 --nav-wait 或重启 SITL 后重试"
    )
    return False


def get_home_position(logger: OutputLogger, master, timeout: float = 10.0) -> Tuple[float, float]:
    logger.log("[位置] 等待 Home 坐标...")
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
        else:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
        logger.log(f"[位置] Home | lat={lat:.7f}°, lon={lon:.7f}°")
        return lat, lon

    lat, lon = -35.363261, 149.165230
    logger.log(f"[位置] 超时，使用 SITL 默认 Home | lat={lat:.7f}°, lon={lon:.7f}°")
    return lat, lon


def get_current_position(master) -> Optional[Tuple[float, float, float]]:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg is None:
        return None
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


def log_fence_plan(
    logger: OutputLogger,
    home_lat: float,
    home_lon: float,
    center_lat: float,
    center_lon: float,
    exit_lat: float,
    exit_lon: float,
    fly_lat: float,
    fly_lon: float,
    center_north_m: float,
    radius_m: float,
    fly_target_north_m: float,
    alt_m: float,
):
    entry_north_m = center_north_m - radius_m
    logger.log("[围栏] 规划参数：")
    logger.log("-" * 72)
    logger.log(f"  Home          | lat={home_lat:.7f}°, lon={home_lon:.7f}°")
    logger.log(
        f"  圆心（Home+{center_north_m:.0f}m北） | "
        f"lat={center_lat:.7f}°, lon={center_lon:.7f}°"
    )
    logger.log(f"  半径          | {radius_m:.0f} m")
    logger.log(
        f"  南边界（进入侧） | Home 北 {entry_north_m:.0f} m 处"
    )
    logger.log(
        f"  退出点（南边界） | lat={exit_lat:.7f}°, lon={exit_lon:.7f}° "
        f"（圆心南 {radius_m:.0f} m）"
    )
    logger.log(
        f"  测试北飞目标   | Home 北 {fly_target_north_m:.0f} m | "
        f"lat={fly_lat:.7f}°, lon={fly_lon:.7f}°, alt={alt_m:.1f}m"
    )
    logger.log("-" * 72)


def refresh_vehicle_state(master, timeout: float = 2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_srcSystem() == master.target_system:
            return msg
    return None


def log_statustext(logger: OutputLogger, master, duration: float = 0.3):
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


def wait_command_ack(master, command: int, timeout: float = COMMAND_ACK_TIMEOUT_SEC):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.command == command:
            return msg
    return None


def send_command_long(
    logger: OutputLogger,
    master,
    command: int,
    label: str,
    params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    wait_ack: bool = True,
):
    p1, p2, p3, p4, p5, p6, p7 = params
    logger.log(f"[命令] {label}")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        p1, p2, p3, p4, p5, p6, p7,
    )
    if not wait_ack:
        return None
    ack = wait_command_ack(master, command)
    if ack is None:
        logger.log(f"[命令] {label} 未收到 COMMAND_ACK")
        return None
    logger.log(f"[命令] {label} ACK | result={mav_result_name(ack.result)}")
    log_statustext(logger, master)
    return ack


def send_command_int(
    logger: OutputLogger,
    master,
    command: int,
    label: str,
    lat: float,
    lon: float,
    alt_m: float,
    params=(0.0, 0.0, 0.0, 0.0),
    wait_ack: bool = True,
):
    p1, p2, p3, p4 = params
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    logger.log(f"[命令] {label}")
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        frame,
        command,
        0,
        0,
        p1,
        p2,
        p3,
        p4,
        int(lat * 1e7),
        int(lon * 1e7),
        float(alt_m),
    )
    if not wait_ack:
        return None
    ack = wait_command_ack(master, command)
    if ack is None:
        logger.log(f"[命令] {label} 未收到 COMMAND_ACK")
        return None
    logger.log(f"[命令] {label} ACK | result={mav_result_name(ack.result)}")
    log_statustext(logger, master)
    return ack


def resolve_mode_number(master, mode_name: str) -> Optional[int]:
    mapping = master.mode_mapping()
    if mapping and mode_name in mapping:
        return mapping[mode_name]
    return {"STABILIZE": 0, "GUIDED": 4, "LOITER": 5, "RTL": 6}.get(mode_name.upper())


def wait_for_flight_mode(logger: OutputLogger, master, mode_name: str, timeout: float) -> bool:
    target = mode_name.upper()
    deadline = time.time() + timeout
    while time.time() < deadline:
        refresh_vehicle_state(master, timeout=1.0)
        mode = getattr(master, "flightmode", None) or "UNKNOWN"
        if target in mode.upper():
            logger.log(f"[状态] 当前模式: {mode}")
            return True
        time.sleep(0.5)
    logger.log(f"[状态] 切换 {mode_name} 超时，当前: {getattr(master, 'flightmode', 'UNKNOWN')}")
    return False


def set_flight_mode(logger: OutputLogger, master, mode_name: str) -> bool:
    mode_number = resolve_mode_number(master, mode_name)
    if mode_number is None:
        logger.log(f"[状态] 未知模式 {mode_name}")
        return False

    current = getattr(master, "flightmode", "") or ""
    if mode_name.upper() in current.upper():
        logger.log(f"[状态] 已在 {mode_name} 模式")
        return True

    logger.log(f"[状态] 切换模式 -> {mode_name}")
    master.set_mode_apm(mode_name)
    if wait_for_flight_mode(logger, master, mode_name, MODE_CHANGE_TIMEOUT_SEC):
        return True

    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if master.motors_armed():
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    master.mav.set_mode_send(master.target_system, base_mode, mode_number)
    return wait_for_flight_mode(logger, master, mode_name, MODE_CHANGE_TIMEOUT_SEC)


def prepare_armable_mode(logger: OutputLogger, master) -> bool:
    refresh_vehicle_state(master, timeout=2.0)
    mode = getattr(master, "flightmode", None) or "UNKNOWN"
    if "STABILIZE" in mode.upper():
        return True
    logger.log(f"[准备] 当前 {mode}，先切 STABILIZE")
    return set_flight_mode(logger, master, "STABILIZE")


def step_arm(logger: OutputLogger, master) -> bool:
    logger.log("[步骤] 解锁")
    if master.motors_armed():
        logger.log("[状态] 已解锁")
        return True

    for attempt in range(1, ARM_MAX_ATTEMPTS + 1):
        refresh_vehicle_state(master, timeout=1.0)
        mode = getattr(master, "flightmode", "") or "UNKNOWN"
        if any(tag in mode.upper() for tag in NON_ARMABLE_MODES):
            if not set_flight_mode(logger, master, "STABILIZE"):
                return False

        ack = send_command_long(
            logger,
            master,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            f"ARM 尝试 {attempt}/{ARM_MAX_ATTEMPTS}",
            (1, 0, 0, 0, 0, 0, 0),
        )
        if ack is not None and not ack_ok(ack.result):
            log_statustext(logger, master, duration=1.0)
            time.sleep(1.0)
            continue

        refresh_vehicle_state(master, timeout=2.0)
        if master.motors_armed():
            logger.log("[状态] 解锁成功")
            return True
        time.sleep(1.0)

    logger.log("[错误] 解锁失败")
    return False


def wait_for_altitude(logger: OutputLogger, master, target_alt_m: float) -> bool:
    logger.log(f"[状态] 等待高度 {target_alt_m:.1f}m ...")
    deadline = time.time() + TAKEOFF_TIMEOUT_SEC
    last_log = 0.0
    while time.time() < deadline:
        pos = get_current_position(master)
        if pos is None:
            time.sleep(0.2)
            continue
        _, _, rel_alt = pos
        if time.time() - last_log >= 2.0:
            logger.log(f"[位置] relative_alt={rel_alt:.2f}m")
            last_log = time.time()
        if rel_alt >= target_alt_m - ALTITUDE_TOLERANCE_M:
            logger.log(f"[状态] 到达高度 {rel_alt:.2f}m")
            return True
        time.sleep(0.1)
    return False


def step_takeoff(logger: OutputLogger, master, alt_m: float) -> bool:
    ack = send_command_long(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        f"TAKEOFF {alt_m:.1f}m",
        (0, 0, 0, 0, 0, 0, alt_m),
    )
    if ack is not None and not ack_ok(ack.result):
        return False
    return wait_for_altitude(logger, master, alt_m)


def send_goto(logger: OutputLogger, master, label: str, lat: float, lon: float, alt_m: float):
    logger.log(f"[导航] 飞往 {label} | lat={lat:.7f}°, lon={lon:.7f}°")
    send_command_int(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        f"DO_REPOSITION -> {label}",
        lat,
        lon,
        alt_m,
        params=(-1, 0, 0, 0),
    )


def wait_for_position(
    logger: OutputLogger,
    master,
    target_lat: float,
    target_lon: float,
    target_alt_m: float,
    label: str,
    timeout: float = GOTO_TIMEOUT_SEC,
) -> bool:
    deadline = time.time() + timeout
    last_log = 0.0
    while time.time() < deadline:
        pos = get_current_position(master)
        if pos is None:
            time.sleep(0.1)
            continue
        lat, lon, rel_alt = pos
        dist = horizontal_distance_m(lat, lon, target_lat, target_lon)
        if time.time() - last_log >= 2.0:
            logger.log(f"[位置] {label} | dist={dist:.1f}m, alt={rel_alt:.1f}m")
            last_log = time.time()
        if dist <= POSITION_TOLERANCE_M and rel_alt >= target_alt_m - ALTITUDE_TOLERANCE_M:
            logger.log(f"[状态] 到达 {label} | dist={dist:.1f}m")
            return True
        time.sleep(0.1)
    logger.log(f"[错误] 到达 {label} 超时")
    return False


def fly_north_until_fence_entry(
    logger: OutputLogger,
    master,
    center_lat: float,
    center_lon: float,
    radius_m: float,
    fly_lat: float,
    fly_lon: float,
    alt_m: float,
    timeout: float,
) -> Optional[Tuple[float, float]]:
    """
    向北飞行并监测是否进入围栏。
    返回触发时的 (lat, lon)，未进入则返回 None。
    """
    send_goto(logger, master, "北飞测试目标", fly_lat, fly_lon, alt_m)
    logger.log("[监测] 向北飞行，等待进入围栏...")

    deadline = time.time() + timeout
    last_log = 0.0
    entered = False
    entry_lat = entry_lon = 0.0

    while time.time() < deadline:
        pos = get_current_position(master)
        if pos is None:
            time.sleep(0.1)
            continue

        lat, lon, rel_alt = pos
        dist_center = horizontal_distance_m(lat, lon, center_lat, center_lon)
        inside = dist_center <= radius_m

        if time.time() - last_log >= 1.0:
            logger.log(
                f"[监测] lat={lat:.7f}°, 距圆心={dist_center:.1f}m, "
                f"围栏内={'是' if inside else '否'}, alt={rel_alt:.1f}m"
            )
            last_log = time.time()

        if inside and not entered:
            entered = True
            entry_lat, entry_lon = lat, lon
            logger.log(
                f"[触发] 进入围栏！| 距圆心={dist_center:.1f}m <= 半径{radius_m:.0f}m"
            )
            logger.log(f"[触发] 进入点 | lat={entry_lat:.7f}°, lon={entry_lon:.7f}°")
            return entry_lat, entry_lon

        if horizontal_distance_m(lat, lon, fly_lat, fly_lon) <= POSITION_TOLERANCE_M:
            if not entered:
                logger.log("[警告] 已到达北飞目标但未检测到进入围栏")
            break

        time.sleep(0.1)

    return None if not entered else (entry_lat, entry_lon)


def handle_fence_breach(
    logger: OutputLogger,
    master,
    exit_lat: float,
    exit_lon: float,
    alt_m: float,
    hover_in_fence_sec: float,
    hover_at_edge_sec: float,
) -> bool:
    """进入围栏后：悬停 → 反向飞出至南边界 → 边界悬停。"""
    logger.log("[响应] 步骤 1/4：围栏内悬停（LOITER）")
    if not set_flight_mode(logger, master, "LOITER"):
        return False

    logger.log(f"[响应] 围栏内悬停 {hover_in_fence_sec:.0f}s ...")
    time.sleep(hover_in_fence_sec)

    logger.log("[响应] 步骤 2/4：切换 GUIDED，反向飞向围栏南边界")
    if not set_flight_mode(logger, master, "GUIDED"):
        return False

    send_goto(logger, master, "围栏南边界（退出点）", exit_lat, exit_lon, alt_m)
    if not wait_for_position(
        logger, master, exit_lat, exit_lon, alt_m, "围栏南边界", GOTO_TIMEOUT_SEC
    ):
        return False

    pos = get_current_position(master)
    if pos:
        lat, lon, _ = pos
        logger.log(
            f"[响应] 步骤 3/4：已到达南边界 | lat={lat:.7f}°, lon={lon:.7f}°"
        )

    logger.log("[响应] 步骤 4/4：边界悬停（LOITER）")
    if not set_flight_mode(logger, master, "LOITER"):
        return False

    logger.log(f"[响应] 边界悬停 {hover_at_edge_sec:.0f}s ...")
    time.sleep(hover_at_edge_sec)
    logger.log("[完成] 围栏触发响应流程结束，飞机在南边界悬停")
    return True


def run_geofence_demo(logger: OutputLogger, master, args) -> bool:
    request_data_streams(logger, master)
    if not wait_for_navigation_ready(
        logger, master, args.nav_wait, DEFAULT_SITL_WARMUP_SEC
    ):
        return False

    home_lat, home_lon = get_home_position(logger, master)

    center_lat, center_lon = offset_position(
        home_lat, home_lon, args.center_north, 0.0
    )
    exit_lat, exit_lon = offset_position(center_lat, center_lon, -args.radius, 0.0)
    fly_lat, fly_lon = offset_position(home_lat, home_lon, args.fly_target_north, 0.0)

    log_fence_plan(
        logger,
        home_lat,
        home_lon,
        center_lat,
        center_lon,
        exit_lat,
        exit_lon,
        fly_lat,
        fly_lon,
        args.center_north,
        args.radius,
        args.fly_target_north,
        args.alt,
    )

    logger.log("[流程] 解锁 -> GUIDED -> 起飞 -> 向北飞 -> 围栏触发 -> 反向至边界")
    logger.log("-" * 72)

    if not prepare_armable_mode(logger, master):
        return False
    if not step_arm(logger, master):
        return False
    if not set_flight_mode(logger, master, "GUIDED"):
        logger.log(
            "[错误] GUIDED 切换失败。常见原因：GPS/EKF 未就绪。"
            "请重启 SITL，等待 20s 后再运行脚本，或增大 --nav-wait"
        )
        return False
    if not step_takeoff(logger, master, args.alt):
        return False

    entry = fly_north_until_fence_entry(
        logger,
        master,
        center_lat,
        center_lon,
        args.radius,
        fly_lat,
        fly_lon,
        args.alt,
        FENCE_MONITOR_TIMEOUT_SEC,
    )
    if entry is None:
        logger.log("[错误] 未进入围栏，请检查 fly-target-north 是否位于围栏内")
        return False

    return handle_fence_breach(
        logger,
        master,
        exit_lat,
        exit_lon,
        args.alt,
        args.hover_in_fence,
        args.hover_at_edge,
    )


def main():
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    with OutputLogger(output_path) as logger:
        logger.log(f"[输出] 日志: {output_path}")

        try:
            master = connect(logger, args.connection, args.timeout)
        except Exception as exc:
            logger.log(f"[错误] 连接失败: {exc}")
            logger.log(
                "[提示] 确认 SITL 已启动且 --out=udp:<WindowsIP>:14551，断开 QGC"
            )
            return 1

        ok = run_geofence_demo(logger, master, args)
        logger.log(f"[输出] 记录已保存: {output_path}")
        return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
