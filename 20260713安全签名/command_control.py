#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
command_control.py - MAVLink 2 命令协议：解锁 / GUIDED / 起飞 / 航点 / 返航

任务要求（第四阶段 · 任务 3）：
  1. 使用 command_long_send() 发送控制命令
  2. 完整流程：解锁 -> GUIDED -> 起飞 10m -> 北 30m -> 东 30m -> RTL
  3. 各步骤等待 COMMAND_ACK 与状态确认
  4. 在 SITL 中完整跑通

使用场景：
  脚本通过 UDP 14551 连接 SITL，独立完成命令控制，无需 QGC。

SITL 启动示例（Ubuntu 虚拟机）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python command_control.py
  python command_control.py --alt 10 --offset 30
  python command_control.py --hold 8
  python command_control.py --output command_log.txt
"""

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_TAKEOFF_ALT_M = 10.0
DEFAULT_OFFSET_M = 30.0
DEFAULT_HOLD_SEC = 5.0
DEFAULT_EXEC_TIMEOUT_SEC = 300.0
COMMAND_ACK_TIMEOUT_SEC = 8.0
PARAM_IO_TIMEOUT_SEC = 5.0
MODE_CHANGE_TIMEOUT_SEC = 15.0
TAKEOFF_TIMEOUT_SEC = 60.0
GOTO_TIMEOUT_SEC = 120.0
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
    """日志记录器：stderr 显示 + 文件写入。"""

    def __init__(self, filepath: Path, title: str):
        self.filepath = filepath.resolve()
        self._header_written = False
        self._file = self.filepath.open("w", encoding="utf-8", newline="\n")
        self._write_header(title)

    def _write_header(self, title: str):
        if self._header_written:
            return
        lines = [
            title,
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
        description="MAVLink 2 命令协议：解锁 / GUIDED / 起飞 / 航点 / 返航"
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
        default=DEFAULT_TAKEOFF_ALT_M,
        help=f"任务高度（米，相对 Home），默认 {DEFAULT_TAKEOFF_ALT_M}",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=DEFAULT_OFFSET_M,
        help=f"航点相对 Home 的偏移距离（米），默认 {DEFAULT_OFFSET_M}",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=DEFAULT_HOLD_SEC,
        help=f"到达目标高度后悬停时间（秒），默认 {DEFAULT_HOLD_SEC}",
    )
    parser.add_argument(
        "--exec-timeout",
        type=float,
        default=DEFAULT_EXEC_TIMEOUT_SEC,
        help=f"返航监控超时（秒），默认 {DEFAULT_EXEC_TIMEOUT_SEC:.0f}",
    )
    parser.add_argument(
        "--keep-params",
        action="store_true",
        help="任务结束后保留 RTL_ALT 修改（默认会恢复原值）",
    )
    parser.add_argument(
        "--output",
        default="",
        help="日志文件路径；默认自动生成 command_output_时间戳.txt",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"command_output_{stamp}.txt"


def mav_result_name(result: int) -> str:
    return MAV_RESULT_NAMES.get(result, str(result))


def ack_ok(result: int) -> bool:
    return result in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )


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


def horizontal_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """平面近似计算两点水平距离（米）。"""
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def log_flight_plan(
    logger: OutputLogger,
    home_lat: float,
    home_lon: float,
    lat_a: float,
    lon_a: float,
    lat_b: float,
    lon_b: float,
    alt_m: float,
    offset_m: float,
):
    logger.log("[航线] GUIDED 命令飞行规划：")
    logger.log("-" * 72)
    logger.log(f"  Home  | lat={home_lat:.7f}°, lon={home_lon:.7f}°")
    logger.log(f"  起飞  | alt={alt_m:.1f}m（相对 Home）")
    logger.log(
        f"  航点 A | 向北 {offset_m:.0f}m | lat={lat_a:.7f}°, lon={lon_a:.7f}°, alt={alt_m:.1f}m"
    )
    logger.log(
        f"  航点 B | 再向东 {offset_m:.0f}m | lat={lat_b:.7f}°, lon={lon_b:.7f}°, alt={alt_m:.1f}m"
    )
    logger.log("  返航  | MAV_CMD_NAV_RETURN_TO_LAUNCH")
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


def apply_rtl_alt(logger: OutputLogger, master, rtl_alt_m: float):
    """将 RTL_ALT 设为与起飞高度一致，返回原值供恢复。"""
    rtl_alt_cm = rtl_alt_m * 100.0
    logger.log("[飞行参数] 调整返航高度：")
    logger.log(
        f"  RTL_ALT={rtl_alt_cm:g} cm ({rtl_alt_m:.1f} m) 返航高度（与 --alt 一致）"
    )
    msg = read_param(logger, master, "RTL_ALT")
    original = (msg.param_value, msg.param_type)
    if abs(msg.param_value - rtl_alt_cm) <= 1e-3:
        logger.log("[飞行参数] RTL_ALT 已是目标值，跳过写入")
        return original
    set_param_value(logger, master, "RTL_ALT", rtl_alt_cm, msg.param_type)
    return original


def restore_rtl_alt(logger: OutputLogger, master, original):
    value, param_type = original
    logger.log(f"[飞行参数] 恢复 RTL_ALT 原值 {value:g} cm ...")
    set_param_value(logger, master, "RTL_ALT", value, param_type)
    logger.log("[飞行参数] RTL_ALT 已恢复")


def wait_command_ack(
    master,
    command: int,
    timeout: float = COMMAND_ACK_TIMEOUT_SEC,
):
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
    logger.log(f"[命令] command_long_send -> {label}")
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
        logger.log(f"[命令] {label} 未收到 COMMAND_ACK（{COMMAND_ACK_TIMEOUT_SEC}s）")
        return None

    logger.log(
        f"[命令] {label} COMMAND_ACK | result={mav_result_name(ack.result)}"
    )
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
    """
    发送 COMMAND_INT（带整数经纬度）。

    ArduPilot 对 DO_REPOSITION 等位置命令只处理 COMMAND_INT，
    用 COMMAND_LONG 会返回 UNSUPPORTED。
    """
    p1, p2, p3, p4 = params
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    logger.log(f"[命令] command_int_send -> {label}")
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
        logger.log(f"[命令] {label} 未收到 COMMAND_ACK（{COMMAND_ACK_TIMEOUT_SEC}s）")
        return None

    logger.log(
        f"[命令] {label} COMMAND_ACK | result={mav_result_name(ack.result)}"
    )
    log_statustext(logger, master)
    return ack


def resolve_mode_number(master, mode_name: str) -> Optional[int]:
    mapping = master.mode_mapping()
    if mapping and mode_name in mapping:
        return mapping[mode_name]
    fallback = {"STABILIZE": 0, "GUIDED": 4, "RTL": 6, "AUTO": 3}
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
            logger.log(f"[状态] 当前模式: {mode}")
            return True
        time.sleep(0.5)
    logger.log(
        f"[状态] 切换 {mode_name} 超时，当前模式: "
        f"{getattr(master, 'flightmode', 'UNKNOWN')}"
    )
    return False


def set_flight_mode(logger: OutputLogger, master, mode_name: str) -> bool:
    mode_number = resolve_mode_number(master, mode_name)
    if mode_number is None:
        logger.log(f"[状态] 未知模式 {mode_name}")
        return False

    logger.log(f"[状态] 切换飞行模式 -> {mode_name}（mode={mode_number}）")
    if mode_name.upper() in (getattr(master, "flightmode", "") or "").upper():
        logger.log(f"[状态] 已在 {mode_name} 模式 | mode={master.flightmode}")
        return True

    master.set_mode_apm(mode_name)
    if wait_for_flight_mode(logger, master, mode_name, MODE_CHANGE_TIMEOUT_SEC / 3):
        return True

    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if master.motors_armed():
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    master.mav.set_mode_send(master.target_system, base_mode, mode_number)
    if wait_for_flight_mode(logger, master, mode_name, MODE_CHANGE_TIMEOUT_SEC / 3):
        return True

    ack = send_command_long(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        f"DO_SET_MODE -> {mode_name}",
        (
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_number,
            0, 0, 0, 0, 0,
        ),
    )
    if ack is not None and not ack_ok(ack.result):
        return False
    return wait_for_flight_mode(logger, master, mode_name, MODE_CHANGE_TIMEOUT_SEC / 3)


def prepare_armable_mode(logger: OutputLogger, master) -> bool:
    """
    解锁前先切 STABILIZE。

    与 mission_upload.py 一致：AUTO/RTL/GUIDED 等模式下解锁易失败，
    第二次连续运行脚本时尤其需要先回到 STABILIZE。
    """
    refresh_vehicle_state(master, timeout=2.0)
    mode = getattr(master, "flightmode", None) or "UNKNOWN"
    if "STABILIZE" in mode.upper():
        logger.log(f"[准备] 当前模式 {mode}，可直接解锁")
        return True

    logger.log(f"[准备] 当前模式 {mode}，先切换 STABILIZE 再解锁")
    return set_flight_mode(logger, master, "STABILIZE")


def set_guided_mode(logger: OutputLogger, master) -> bool:
    logger.log("[步骤 2] 切换 GUIDED 模式")
    return set_flight_mode(logger, master, "GUIDED")


def step_arm(logger: OutputLogger, master) -> bool:
    logger.log("[步骤 1] 解锁（Arm）")
    if master.motors_armed():
        logger.log("[状态] 飞行器已解锁")
        return True

    for attempt in range(1, ARM_MAX_ATTEMPTS + 1):
        refresh_vehicle_state(master, timeout=1.0)
        mode = getattr(master, "flightmode", "") or "UNKNOWN"
        logger.log(f"[步骤 1] 解锁尝试 {attempt}/{ARM_MAX_ATTEMPTS} | 当前模式={mode}")

        if any(tag in mode.upper() for tag in NON_ARMABLE_MODES):
            logger.log(f"[步骤 1] {mode} 不可解锁，切换 STABILIZE")
            if not set_flight_mode(logger, master, "STABILIZE"):
                return False

        ack = send_command_long(
            logger,
            master,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            "MAV_CMD_COMPONENT_ARM_DISARM (arm=1)",
            (1, 0, 0, 0, 0, 0, 0),
        )
        if ack is not None and not ack_ok(ack.result):
            logger.log(
                f"[步骤 1] 解锁 ACK={mav_result_name(ack.result)}，"
                "读取飞控提示并重试..."
            )
            log_statustext(logger, master, duration=1.5)
            if not set_flight_mode(logger, master, "STABILIZE"):
                return False
            time.sleep(1.0)
            continue

        refresh_vehicle_state(master, timeout=2.0)
        if master.motors_armed():
            logger.log("[状态] 解锁成功")
            return True
        time.sleep(1.0)

    log_statustext(logger, master, duration=1.5)
    logger.log(
        "[错误] 解锁失败，请检查：\n"
        "  1. 上方 [飞控] PreArm 提示\n"
        "  2. 断开 QGC 后重试\n"
        "  3. 重启 SITL（连续跑两次任务后建议重启）"
    )
    return False


def get_relative_altitude(master) -> Optional[float]:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


def wait_for_altitude(
    logger: OutputLogger,
    master,
    target_alt_m: float,
    timeout: float = TAKEOFF_TIMEOUT_SEC,
) -> bool:
    logger.log(f"[状态] 等待到达高度 {target_alt_m:.1f}m ...")
    deadline = time.time() + timeout
    last_log = 0.0

    while time.time() < deadline:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        rel_alt = msg.relative_alt / 1000.0
        now = time.time()
        if now - last_log >= 2.0:
            logger.log(f"[位置] relative_alt={rel_alt:.2f}m")
            last_log = now
        if rel_alt >= target_alt_m - ALTITUDE_TOLERANCE_M:
            logger.log(f"[状态] 已到达目标高度 | relative_alt={rel_alt:.2f}m")
            return True

    logger.log("[错误] 等待起飞高度超时")
    return False


def wait_for_position(
    logger: OutputLogger,
    master,
    target_lat: float,
    target_lon: float,
    target_alt_m: float,
    label: str,
    timeout: float = GOTO_TIMEOUT_SEC,
) -> bool:
    logger.log(
        f"[状态] 等待到达 {label} | lat={target_lat:.7f}°, lon={target_lon:.7f}°, "
        f"alt={target_alt_m:.1f}m ..."
    )
    deadline = time.time() + timeout
    last_log = 0.0

    while time.time() < deadline:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg is None:
            continue

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        rel_alt = msg.relative_alt / 1000.0
        dist = horizontal_distance_m(lat, lon, target_lat, target_lon)
        now = time.time()
        if now - last_log >= 2.0:
            logger.log(
                f"[位置] {label} | dist={dist:.1f}m, relative_alt={rel_alt:.2f}m"
            )
            last_log = now

        if (
            dist <= POSITION_TOLERANCE_M
            and rel_alt >= target_alt_m - ALTITUDE_TOLERANCE_M
        ):
            logger.log(
                f"[状态] 已到达 {label} | dist={dist:.1f}m, relative_alt={rel_alt:.2f}m"
            )
            return True

    logger.log(f"[错误] 等待到达 {label} 超时")
    return False


def step_goto(
    logger: OutputLogger,
    master,
    label: str,
    lat: float,
    lon: float,
    alt_m: float,
) -> bool:
    """GUIDED 模式下用 DO_REPOSITION 飞往指定经纬度（COMMAND_INT）。"""
    logger.log(
        f"[步骤] 飞往 {label} | lat={lat:.7f}°, lon={lon:.7f}°, alt={alt_m:.1f}m"
    )
    ack = send_command_int(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        f"MAV_CMD_DO_REPOSITION -> {label}",
        lat,
        lon,
        alt_m,
        params=(-1, 0, 0, 0),
    )
    if ack is not None and not ack_ok(ack.result):
        return False
    return wait_for_position(logger, master, lat, lon, alt_m, label)


def step_takeoff(logger: OutputLogger, master, alt_m: float) -> bool:
    logger.log(f"[步骤 3] 起飞至 {alt_m:.1f} 米")
    ack = send_command_long(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        f"MAV_CMD_NAV_TAKEOFF (alt={alt_m}m)",
        (0, 0, 0, 0, 0, 0, alt_m),
    )
    if ack is not None and not ack_ok(ack.result):
        return False
    return wait_for_altitude(logger, master, alt_m)


def step_rtl(logger: OutputLogger, master) -> bool:
    logger.log("[步骤 6] 返航（RTL）")
    ack = send_command_long(
        logger,
        master,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        "MAV_CMD_NAV_RETURN_TO_LAUNCH",
    )
    if ack is not None and not ack_ok(ack.result):
        return False

    refresh_vehicle_state(master, timeout=2.0)
    mode = getattr(master, "flightmode", "") or "UNKNOWN"
    if "RTL" in mode.upper():
        logger.log(f"[状态] 已进入 RTL 模式 | mode={mode}")
        return True

    return wait_for_flight_mode(logger, master, "RTL", MODE_CHANGE_TIMEOUT_SEC)


def monitor_rtl(logger: OutputLogger, master, timeout: float) -> bool:
    logger.log(f"[监控] 等待返航完成（超时 {timeout:.0f}s）")
    logger.log("-" * 72)

    start = time.time()
    last_log = 0.0
    saw_rtl = False

    while time.time() - start < timeout:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT" and msg.get_srcSystem() == master.target_system:
            mode = mavutil.mode_string_v10(msg)
            if "RTL" in mode.upper():
                saw_rtl = True
            if saw_rtl and not master.motors_armed():
                logger.log(f"[进度] 返航完成，已上锁 | mode={mode}")
                return True

        elif msg_type == "GLOBAL_POSITION_INT":
            now = time.time()
            if now - last_log >= 3.0:
                rel_alt = msg.relative_alt / 1000.0
                logger.log(f"[位置] relative_alt={rel_alt:.2f}m")
                last_log = now

    logger.log("[监控] 返航监控超时，请手动查看 SITL 状态")
    return False


def run_command_sequence(
    logger: OutputLogger,
    master,
    home_lat: float,
    home_lon: float,
    alt_m: float,
    offset_m: float,
    hold_sec: float,
    exec_timeout: float,
    keep_params: bool = False,
) -> bool:
    lat_a, lon_a = offset_position(home_lat, home_lon, offset_m, 0.0)
    lat_b, lon_b = offset_position(home_lat, home_lon, offset_m, offset_m)
    log_flight_plan(logger, home_lat, home_lon, lat_a, lon_a, lat_b, lon_b, alt_m, offset_m)

    logger.log(
        "[流程] 开始命令控制序列：解锁 -> GUIDED -> 起飞 -> 航点A -> 航点B -> RTL"
    )
    logger.log("-" * 72)

    rtl_original = None
    try:
        if not prepare_armable_mode(logger, master):
            return False
        if not step_arm(logger, master):
            return False
        if not set_guided_mode(logger, master):
            return False
        if not step_takeoff(logger, master, alt_m):
            return False

        if hold_sec > 0:
            logger.log(f"[状态] 悬停 {hold_sec:.0f}s ...")
            time.sleep(hold_sec)

        if not step_goto(logger, master, "航点 A（北）", lat_a, lon_a, alt_m):
            return False
        if not step_goto(logger, master, "航点 B（东）", lat_b, lon_b, alt_m):
            return False

        rtl_original = apply_rtl_alt(logger, master, alt_m)

        if not step_rtl(logger, master):
            return False

        monitor_rtl(logger, master, exec_timeout)
        logger.log("[流程] 命令控制序列结束")
        return True
    finally:
        if rtl_original is not None and not keep_params:
            restore_rtl_alt(logger, master, rtl_original)


def main():
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    with OutputLogger(output_path, "MAVLink 2 命令协议控制记录") as logger:
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
        ok = run_command_sequence(
            logger,
            master,
            home_lat,
            home_lon,
            args.alt,
            args.offset,
            args.hold,
            args.exec_timeout,
            args.keep_params,
        )

        logger.log(f"[输出] 记录已保存至: {output_path}")
        return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
