#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
basic_communication.py - Python 与 MAVLink 2 基础交互

任务要求：
  1. 建立与 SITL 仿真器的 MAVLink 连接
  2. 接收并解析 HEARTBEAT，打印 sysid / compid
  3. 请求姿态与位置数据流
  4. 解析 ATTITUDE、GLOBAL_POSITION_INT，打印欧拉角与经纬高

运行前请确保 SITL 已启动。Windows 接收虚拟机 SITL 时，启动命令示例：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14550

用法：
  python basic_communication.py
  python basic_communication.py --connection udpin:0.0.0.0:14550
  python basic_communication.py --connection udp:127.0.0.1:14550
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(description="MAVLink 2 基础通信示例")
    parser.add_argument(
        "--connection",
        default="udpin:0.0.0.0:14550",
        help=(
            "MAVLink 连接字符串。"
            "Windows 接收虚拟机 SITL 用 udpin:0.0.0.0:14550；"
            "脚本与 SITL 同机可用 udp:127.0.0.1:14550"
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="等待心跳的超时时间（秒），默认 10",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="持续接收并打印数据的时长（秒），默认 30；设为 0 表示一直运行",
    )
    return parser.parse_args()


def connect(connection_string: str, timeout: float):
    """建立 MAVLink 连接并等待心跳。"""
    print(f"[连接] 正在连接: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)

    print("[连接] 等待 HEARTBEAT ...")
    master.wait_heartbeat(timeout=timeout)
    print(
        f"[连接] 已收到心跳 | target_system={master.target_system}, "
        f"target_component={master.target_component}"
    )
    return master


def print_heartbeat(msg):
    """解析并打印 HEARTBEAT 关键字段。"""
    print(
        "[HEARTBEAT] "
        f"sysid={msg.get_srcSystem()}, "
        f"compid={msg.get_srcComponent()}, "
        f"type={msg.type}, "
        f"autopilot={msg.autopilot}, "
        f"base_mode={msg.base_mode}, "
        f"custom_mode={msg.custom_mode}, "
        f"system_status={msg.system_status}"
    )


def request_data_streams(master):
    """
    向飞控请求姿态与位置数据流。

    优先使用 MAVLink 2 的 SET_MESSAGE_INTERVAL（ArduPilot SITL 兼容更好），
    同时发送 REQUEST_DATA_STREAM 以满足任务要求。
    """
    target_system = master.target_system
    target_component = master.target_component

    # 方式一：MAVLink 2 按消息 ID 设置发送间隔（微秒），4Hz = 250000us
    intervals = {
        "ATTITUDE": (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 250_000),
        "GLOBAL_POSITION_INT": (
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            250_000,
        ),
    }
    for name, (msg_id, interval_us) in intervals.items():
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        print(f"[请求] SET_MESSAGE_INTERVAL -> {name} @ 4Hz")

    # 方式二：经典 REQUEST_DATA_STREAM（任务要求）
    stream_requests = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, "姿态(ATTITUDE)"),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, "位置(GLOBAL_POSITION_INT)"),
    ]
    for stream_id, label in stream_requests:
        master.mav.request_data_stream_send(
            target_system,
            target_component,
            stream_id,
            4,  # 4 Hz
            1,  # 开启
        )
        print(f"[请求] REQUEST_DATA_STREAM -> {label}")


def format_attitude(msg):
    """将 ATTITUDE 消息格式化为可读字符串（弧度转角度）。"""
    roll_deg = math.degrees(msg.roll)
    pitch_deg = math.degrees(msg.pitch)
    yaw_deg = math.degrees(msg.yaw)
    return (
        f"roll={roll_deg:7.2f}°, "
        f"pitch={pitch_deg:7.2f}°, "
        f"yaw={yaw_deg:7.2f}°"
    )


def format_global_position(msg):
    """将 GLOBAL_POSITION_INT 消息格式化为可读字符串。"""
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt_m = msg.alt / 1000.0
    relative_alt_m = msg.relative_alt / 1000.0
    return (
        f"lat={lat:.7f}°, "
        f"lon={lon:.7f}°, "
        f"alt={alt_m:.2f}m, "
        f"relative_alt={relative_alt_m:.2f}m"
    )


def main():
    args = parse_args()

    try:
        master = connect(args.connection, args.timeout)
    except Exception as exc:
        print(f"[错误] 连接失败: {exc}", file=sys.stderr)
        print(
            "[提示] 请确认 SITL 已启动，且 --out 指向本机 IP:14550；"
            "若 QGC 占用 14550，可为 Python 使用 14551 端口。",
            file=sys.stderr,
        )
        return 1

    # 首次专门接收并打印 HEARTBEAT
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb is not None:
        print_heartbeat(hb)
    else:
        print("[警告] 连接后未再次收到 HEARTBEAT")

    # 请求数据流
    request_data_streams(master)
    time.sleep(0.5)

    print("[接收] 开始监听 ATTITUDE / GLOBAL_POSITION_INT（Ctrl+C 退出）")
    print("-" * 72)

    start = time.time()
    try:
        while True:
            if args.duration > 0 and (time.time() - start) >= args.duration:
                print(f"[结束] 已运行 {args.duration:.0f} 秒，退出")
                break

            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                print_heartbeat(msg)
            elif msg_type == "ATTITUDE":
                print(f"[ATTITUDE]            {format_attitude(msg)}")
            elif msg_type == "GLOBAL_POSITION_INT":
                print(f"[GLOBAL_POSITION_INT] {format_global_position(msg)}")

    except KeyboardInterrupt:
        print("\n[结束] 用户中断")

    return 0


if __name__ == "__main__":
    sys.exit(main())
