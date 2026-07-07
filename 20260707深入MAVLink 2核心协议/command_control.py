#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
command_control.py - MAVLink 2 命令协议：解锁 / GUIDED / 起飞 / 返航

任务要求（第四阶段 · 任务 3）：
  1. 使用 command_long_send() 发送控制命令
  2. 完整流程：解锁 -> 切换 GUIDED -> 起飞至 10 米 -> 返航（RTL）
  3. 各步骤等待 COMMAND_ACK 与状态确认
  4. 在 SITL 中完整跑通

使用场景：
  脚本通过 UDP 14551 连接 SITL，独立完成命令控制，无需 QGC。

SITL 启动示例（Ubuntu 虚拟机）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python command_control.py
  python command_control.py --alt 10
  python command_control.py --hold 8
  python command_control.py --output command_log.txt
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_TAKEOFF_ALT_M = 10.0
DEFAULT_HOLD_SEC = 5.0
DEFAULT_EXEC_TIMEOUT_SEC = 300.0
COMMAND_ACK_TIMEOUT_SEC = 8.0
PARAM_IO_TIMEOUT_SEC = 5.0
MODE_CHANGE_TIMEOUT_SEC = 15.0
TAKEOFF_TIMEOUT_SEC = 60.0
ALTITUDE_TOLERANCE_M = 0.8
ARM_MAX_ATTEMPTS = 5
NON_ARMABLE_MODES = ("AUTO", "RTL", "LAND", "LOITER")

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
        description="MAVLink 2 命令协议：解锁 / GUIDED / 起飞 / 返航"
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
        help=f"起飞高度（米），默认 {DEFAULT_TAKEOFF_ALT_M}",
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
    若处于 AUTO 等不可解锁模式，先切 STABILIZE。
    （常见于刚运行过 mission_upload.py 后未重置 SITL）
    """
    refresh_vehicle_state(master, timeout=2.0)
    mode = getattr(master, "flightmode", None) or "UNKNOWN"
    if any(tag in mode.upper() for tag in NON_ARMABLE_MODES):
        logger.log(f"[准备] 当前 {mode} 不可解锁，先切换 STABILIZE")
        return set_flight_mode(logger, master, "STABILIZE")
    return True


def set_guided_mode(logger: OutputLogger, master) -> bool:
    logger.log("[步骤 2] 切换 GUIDED 模式")
    return set_flight_mode(logger, master, "GUIDED")


def step_arm(logger: OutputLogger, master) -> bool:
    logger.log("[步骤 1] 解锁（Arm）")
    if master.motors_armed():
        logger.log("[状态] 飞行器已解锁")
        return True

    for attempt in range(1, ARM_MAX_ATTEMPTS + 1):
        logger.log(f"[步骤 1] 解锁尝试 {attempt}/{ARM_MAX_ATTEMPTS}")
        ack = send_command_long(
            logger,
            master,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            "MAV_CMD_COMPONENT_ARM_DISARM (arm=1)",
            (1, 0, 0, 0, 0, 0, 0),
        )
        if ack is not None and not ack_ok(ack.result):
            mode = getattr(master, "flightmode", "") or "UNKNOWN"
            if any(tag in mode.upper() for tag in NON_ARMABLE_MODES):
                logger.log(f"[步骤 1] {mode} 不可解锁，切换 STABILIZE 后重试")
                if not set_flight_mode(logger, master, "STABILIZE"):
                    return False
            time.sleep(1.0)
            continue

        refresh_vehicle_state(master, timeout=2.0)
        if master.motors_armed():
            logger.log("[状态] 解锁成功")
            return True
        time.sleep(1.0)

    logger.log("[错误] 解锁失败（可重启 SITL 或断开 QGC 后重试）")
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
    logger.log("[步骤 4] 返航（RTL）")
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
    alt_m: float,
    hold_sec: float,
    exec_timeout: float,
    keep_params: bool = False,
) -> bool:
    logger.log("[流程] 开始命令控制序列：解锁 -> GUIDED -> 起飞 -> RTL")
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

        ok = run_command_sequence(
            logger,
            master,
            args.alt,
            args.hold,
            args.exec_timeout,
            args.keep_params,
        )

        logger.log(f"[输出] 记录已保存至: {output_path}")
        return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
