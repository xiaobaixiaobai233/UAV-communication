#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
basic_communication.py - Python 与 MAVLink 2 基础交互

使用场景：
  - QGC（地面站）通过 UDP 14550 控制虚拟机中的 SITL 仿真飞机
  - 本脚本通过 UDP 14551 监听同一架飞机的 MAVLink 数据，并写入日志文件

任务要求：
  1. 建立与 SITL 仿真器的 MAVLink 连接
  2. 接收并解析 HEARTBEAT，打印 sysid / compid
  3. 请求姿态与位置数据流
  4. 解析 ATTITUDE、GLOBAL_POSITION_INT，打印欧拉角与经纬高

SITL 启动示例（Ubuntu 虚拟机，双端口输出）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14550 \\
      --out=udp:<Windows主机IP>:14551

用法：
  python basic_communication.py
  python basic_communication.py --duration 0
  python basic_communication.py --output mavlink_output.txt
"""

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

# 本脚本默认监听端口（QGC 使用 14550，脚本使用 14551，避免端口冲突）
DEFAULT_LISTEN_PORT = 14551

#通过脚本输出监听数据
class OutputLogger:
    """日志记录器：终端显示的同时写入 txt 文件，便于提交实验报告。"""

    def __init__(self, filepath: Path):
        self.filepath = filepath
        # 以 UTF-8 打开文件，避免中文乱码
        self._file = filepath.open("w", encoding="utf-8")
        self._write_header()

    def _write_header(self):
        """在日志文件开头写入标题和开始时间。"""
        header = (
            "MAVLink 2 基础交互输出记录\n"
            f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"监听端口: UDP {DEFAULT_LISTEN_PORT}（QGC 使用 14550）\n"
            "=" * 72 + "\n"
        )
        self._file.write(header)
        self._file.flush()

    def log(self, message: str = ""):
        """同时输出到终端和文件。"""
        print(message)
        self._file.write(message + "\n")
        self._file.flush()

    def close(self):
        """在日志文件末尾写入结束时间并关闭文件。"""
        footer = (
            "=" * 72 + "\n"
            f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        )
        self._file.write(footer)
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def parse_args():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="监听 SITL 飞行器状态（配合 QGC 控制使用）"
    )
    parser.add_argument(
        "--connection",
        default=f"udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}",
        help=(
            f"MAVLink 连接字符串，默认 udpin:0.0.0.0:{DEFAULT_LISTEN_PORT}。"
            "udpin 表示在本机指定端口监听 SITL 发来的数据"
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
        default=0,
        help="持续监听的时长（秒），默认 0 表示一直运行直到 Ctrl+C",
    )
    parser.add_argument(
        "--output",
        default="",
        help="输出文档路径；默认自动生成 mavlink_output_时间戳.txt",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    """生成带时间戳的默认输出文件名。"""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"mavlink_output_{stamp}.txt"


def connect(logger: OutputLogger, connection_string: str, timeout: float):
    """
    建立 MAVLink 连接并等待第一帧心跳。

    master 是 pymavlink 的连接对象，后续通过它收消息、发请求。
    wait_heartbeat() 会阻塞，直到收到 HEARTBEAT 或超时。
    """
    logger.log(f"[连接] 正在监听: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)

    logger.log("[连接] 等待 HEARTBEAT（请确认 SITL 已启动且 --out 包含 14551）...")
    master.wait_heartbeat(timeout=timeout)

    # target_system / target_component 是 pymavlink 从心跳中自动识别的目标地址
    logger.log(
        f"[连接] 已收到心跳 | target_system={master.target_system}, "
        f"target_component={master.target_component}"
    )
    return master


def log_heartbeat(logger: OutputLogger, msg):
    """
    解析并输出 HEARTBEAT（心跳）消息。

    关键字段：
      sysid   - 系统 ID，标识是哪架飞机
      compid  - 组件 ID，标识飞控等组件
      type    -载具类型（2 = 多旋翼）
      autopilot  -飞控固件（3 = ArduPilot）
      base_mode / custom_mode - 基础模式
      custom_mode  -  当前具体飞行模式
      system_status  -  系统运行状态
    """
    logger.log(
        "[HEARTBEAT] "
        f"sysid={msg.get_srcSystem()}, "       
        f"compid={msg.get_srcComponent()}, "
        f"type={msg.type}, "
        f"autopilot={msg.autopilot}, "
        f"base_mode={msg.base_mode}, "
        f"custom_mode={msg.custom_mode}, "
        f"system_status={msg.system_status}"
    )


def request_data_streams(logger: OutputLogger, master):
    """
    向飞控请求姿态与位置数据流。

    飞机默认不一定持续发送 ATTITUDE / GLOBAL_POSITION_INT，
    需要主动请求后才会以固定频率推送。

    使用两种方式（满足任务要求且兼容 ArduPilot SITL）：
      1. SET_MESSAGE_INTERVAL  - MAVLink 2 推荐方式
      2. REQUEST_DATA_STREAM   - 经典方式
    """
    target_system = master.target_system
    target_component = master.target_component

    # 消息 ID 与发送间隔（微秒），250000us = 4Hz
    message_intervals = {
        "ATTITUDE": (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 250_000),
        "GLOBAL_POSITION_INT": (
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            250_000,
        ),
    }

    for name, (msg_id, interval_us) in message_intervals.items():
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,       # 要订阅的消息 ID
            interval_us,  # 发送间隔（微秒）
            0, 0, 0, 0, 0,
        )
        logger.log(f"[请求] SET_MESSAGE_INTERVAL -> {name} @ 4Hz")

    # 按数据流类型请求（EXTRA1=姿态，POSITION=位置）
    stream_requests = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, "姿态(ATTITUDE)"),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, "位置(GLOBAL_POSITION_INT)"),
    ]
    for stream_id, label in stream_requests:
        master.mav.request_data_stream_send(
            target_system,
            target_component,
            stream_id,
            4,  # 请求频率 4Hz
            1,  # 1=开启，0=关闭
        )
        logger.log(f"[请求] REQUEST_DATA_STREAM -> {label}")


def format_attitude(msg):
    """
    格式化 ATTITUDE 消息。

    原始 roll/pitch/yaw 单位为弧度，这里转换为角度便于阅读。
    在 QGC 中起飞或执行航线后，这些数值会明显变化。
    """
    roll_deg = math.degrees(msg.roll)
    pitch_deg = math.degrees(msg.pitch)
    yaw_deg = math.degrees(msg.yaw)
    return (
        f"roll={roll_deg:7.2f}°, "
        f"pitch={pitch_deg:7.2f}°, "
        f"yaw={yaw_deg:7.2f}°"
    )


def format_global_position(msg):
    """
    格式化 GLOBAL_POSITION_INT 消息。

    lat/lon 在消息中为整数，需除以 1e7 得到度；
    alt / relative_alt 单位为毫米，需除以 1000 得到米。
  relative_alt 为相对起飞点高度，飞机上升时该值会增大。
    """
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
    output_path = Path(args.output) if args.output else default_output_path()

    with OutputLogger(output_path) as logger:
        logger.log(f"[输出] 日志文件: {output_path}")
        logger.log(
            "[说明] 本脚本仅监听数据；请在 QGC 中解锁、起飞或执行航线，"
            "姿态与位置数据会随之变化。"
        )

        # ---------- 第一步：连接 SITL ----------
        try:
            master = connect(logger, args.connection, args.timeout)
        except Exception as exc:
            logger.log(f"[错误] 连接失败: {exc}")
            logger.log(
                "[提示] 请确认：\n"
                "  1. 虚拟机中 SITL 正在运行\n"
                "  2. 启动命令包含 --out=udp:<WindowsIP>:14551\n"
                "  3. QGC 使用 14550，脚本使用 14551，互不冲突"
            )
            return 1

        # ---------- 第二步：解析 HEARTBEAT ----------
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb is not None:
            log_heartbeat(logger, hb)
        else:
            logger.log("[警告] 连接后未再次收到 HEARTBEAT")

        # ---------- 第三步：请求数据流 ----------
        request_data_streams(logger, master)
        time.sleep(0.5)  # 等待飞控处理请求

        # ---------- 第四步：循环监听姿态与位置 ----------
        logger.log("[接收] 开始监听（可在 QGC 中操作飞机，Ctrl+C 退出）")
        logger.log("-" * 72)

        start = time.time()
        try:
            while True:
                # duration > 0 时定时退出；默认 0 表示持续监听
                if args.duration > 0 and (time.time() - start) >= args.duration:
                    logger.log(f"[结束] 已运行 {args.duration:.0f} 秒，退出")
                    break

                # recv_match 阻塞等待下一条消息，timeout 避免死等
                msg = master.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    log_heartbeat(logger, msg)
                elif msg_type == "ATTITUDE":
                    logger.log(f"[ATTITUDE]            {format_attitude(msg)}")
                elif msg_type == "GLOBAL_POSITION_INT":
                    logger.log(
                        f"[GLOBAL_POSITION_INT] {format_global_position(msg)}"
                    )

        except KeyboardInterrupt:
            logger.log("[结束] 用户中断（Ctrl+C）")

        logger.log(f"[输出] 记录已保存至: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
