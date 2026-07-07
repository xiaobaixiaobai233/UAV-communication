#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
param_ops.py - MAVLink 2 参数协议读写

任务要求（第四阶段 · 任务 1）：
  1. 使用 param_request_list() 获取飞控全部参数
  2. 打印并记录参数列表
  3. 读取并修改 1~2 个参数（默认 SR1_EXT_STAT、SR1_EXTRA1）
  4. 读回验证修改是否生效

使用场景：
  脚本通过 UDP 14551 连接 SITL，独立完成参数读写，无需 QGC。

SITL 启动示例（Ubuntu 虚拟机）：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python param_ops.py
  python param_ops.py --list-only          # 仅拉取并列出参数，不修改
  python param_ops.py --output param_log.txt
  python param_ops.py --modify SR1_EXT_STAT=2 SR1_EXTRA1=4

注意：
  SRx_* 流速率参数修改后，通常需重启飞控才会影响实际遥测推送频率；
  本脚本通过 PARAM_VALUE 读回确认写入是否成功。
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_MODIFY_SPECS = ("SR1_EXT_STAT=2", "SR1_EXTRA1=4")#修改的参数
FETCH_TIMEOUT_SEC = 60.0
PARAM_IO_TIMEOUT_SEC = 5.0


class OutputLogger:
    """日志记录器：终端显示的同时写入 txt 文件。

    控制台输出走 stderr，避免 IDE 将 stdout 重定向到与 --output 相同文件时
    与文件写入冲突，导致 header 重复（72 行 '=' 前缀块）。
    """

    def __init__(self, filepath: Path):
        self.filepath = filepath.resolve()
        self._header_written = False
        self._file = self.filepath.open("w", encoding="utf-8", newline="\n")
        self._write_header()

    def _write_header(self):
        if self._header_written:
            return
        lines = [
            "MAVLink 2 参数协议读写记录",
            f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"监听端口: UDP {DEFAULT_LISTEN_PORT}",
            "=" * 72,
        ]
        self._file.write("\n".join(lines) + "\n")
        self._file.flush()
        self._header_written = True

    def log(self, message: str = ""):
        # 使用 stderr，避免 stdout 被重定向到日志文件时重复写入
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
        description="MAVLink 2 参数协议：拉取全部参数并演示读写"
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
        help="等待心跳的超时时间（秒），默认 10",
    )
    parser.add_argument(
        "--fetch-timeout",
        type=float,
        default=FETCH_TIMEOUT_SEC,
        help=f"拉取全部参数的超时时间（秒），默认 {FETCH_TIMEOUT_SEC:.0f}",
    )
    parser.add_argument(
        "--output",
        default="",
        help="日志文件路径；默认自动生成 param_output_时间戳.txt",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="仅拉取并记录全部参数，不执行修改演示",
    )
    parser.add_argument(
        "--keep-changes",
        action="store_true",
        help="修改演示结束后保留新值（默认会恢复原值）",
    )
    parser.add_argument(
        "--modify",
        nargs="*",
        default=list(DEFAULT_MODIFY_SPECS),
        metavar="NAME=VALUE",
        help=(
            "要修改的参数，格式 NAME=VALUE；"
            f"默认 {' '.join(DEFAULT_MODIFY_SPECS)}"
        ),
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"param_output_{stamp}.txt"


def param_id_to_str(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode("utf-8", errors="replace").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def param_id_to_bytes(name: str) -> bytes:
    return name.encode("utf-8")[:16]


def parse_modify_specs(specs):
    """解析 NAME=VALUE 列表，返回 [(name, float_value), ...]。"""
    parsed = []
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"参数格式错误，应为 NAME=VALUE: {spec}")
        name, value_text = spec.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"参数名不能为空: {spec}")
        parsed.append((name, float(value_text.strip())))
    return parsed


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


def fetch_all_params(logger: OutputLogger, master, timeout: float):
    """
    发送 PARAM_REQUEST_LIST，收集全部 PARAM_VALUE。

    返回 dict: {参数名: {"value", "type", "index", "count"}}
    """
    logger.log("[参数] 发送 param_request_list() 请求全部参数...")
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component,
    )

    params = {}
    expected_count = None
    start = time.time()

    while True:
        elapsed = time.time() - start
        if elapsed > timeout:
            logger.log(f"[警告] 拉取参数超时（{timeout:.0f}s），已收到 {len(params)} 条")
            break

        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if expected_count is not None and len(params) >= expected_count:
                break
            continue

        name = param_id_to_str(msg.param_id)
        params[name] = {
            "value": msg.param_value,
            "type": msg.param_type,
            "index": msg.param_index,
            "count": msg.param_count,
        }
        if expected_count is None:
            expected_count = msg.param_count
            logger.log(f"[参数] 飞控报告参数总数: {expected_count}")

        if expected_count and len(params) >= expected_count:
            break

    logger.log(f"[参数] 实际收到 {len(params)} 个参数")
    return params


def log_param_table(logger: OutputLogger, params: dict):
    """按名称排序输出全部参数。"""
    logger.log("[参数] 全部参数列表（按名称排序）：")
    logger.log("-" * 72)
    logger.log(f"{'序号':>6}  {'参数名':<16}  {'值':>12}  {'类型':>4}")
    logger.log("-" * 72)

    for idx, name in enumerate(sorted(params.keys()), start=1):
        entry = params[name]
        logger.log(
            f"{idx:6d}  {name:<16}  {entry['value']:12.6g}  {entry['type']:4d}"
        )

    logger.log("-" * 72)


def request_param_read(logger: OutputLogger, master, param_name: str):
    """发送 PARAM_REQUEST_READ 并等待对应 PARAM_VALUE。"""
    logger.log(f"[读取] param_request_read -> {param_name}")
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
                f"[读取] {param_name} = {msg.param_value:g} "
                f"(type={msg.param_type}, index={msg.param_index})"
            )
            return msg

    raise TimeoutError(f"读取参数 {param_name} 超时（{PARAM_IO_TIMEOUT_SEC}s）")


def set_param(logger: OutputLogger, master, param_name: str, new_value: float, param_type: int):
    """
    发送 PARAM_SET 并等待飞控以 PARAM_VALUE 确认。

    ArduPilot 参数值在 MAVLink 中均以 float 传递。
    """
    logger.log(f"[写入] param_set -> {param_name} = {new_value:g} (type={param_type})")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_to_bytes(param_name),
        float(new_value),
        param_type,
    )

    deadline = time.time() + PARAM_IO_TIMEOUT_SEC
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if param_id_to_str(msg.param_id) != param_name:
            continue
        logger.log(
            f"[确认] {param_name} = {msg.param_value:g} "
            f"(type={msg.param_type}, index={msg.param_index})"
        )
        return msg

    raise TimeoutError(f"写入参数 {param_name} 未收到确认（{PARAM_IO_TIMEOUT_SEC}s）")


def verify_param_change(logger: OutputLogger, master, param_name: str, expected_value: float, tolerance=1e-5):
    """读回参数并与期望值比较。"""
    msg = request_param_read(logger, master, param_name)
    actual = msg.param_value
    if abs(actual - expected_value) <= tolerance:
        logger.log(f"[验证] {param_name} 读回成功，值与期望一致: {actual:g}")
        return True

    logger.log(
        f"[验证] {param_name} 读回值 {actual:g} 与期望 {expected_value:g} 不一致"
    )
    return False


def run_modify_demo(logger: OutputLogger, master, params: dict, modify_specs, keep_changes: bool):
    """演示：读取 -> 修改 -> 读回验证 -> （可选）恢复原值。"""
    logger.log("[演示] 开始参数读写验证")
    logger.log("-" * 72)

    originals = []
    for param_name, target_value in modify_specs:
        if param_name not in params:
            logger.log(f"[警告] 参数 {param_name} 不在飞控参数列表中，跳过")
            continue

        entry = params[param_name]
        original_value = entry["value"]
        param_type = entry["type"]
        originals.append((param_name, original_value, param_type))

        logger.log(f"[演示] 参数 {param_name}: 当前值 = {original_value:g}")

        request_param_read(logger, master, param_name)
        set_param(logger, master, param_name, target_value, param_type)
        verify_param_change(logger, master, param_name, target_value)

    if keep_changes:
        logger.log("[演示] 已保留修改后的参数值（--keep-changes）")
        return

    if not originals:
        return

    logger.log("[演示] 恢复参数原值...")
    for param_name, original_value, param_type in originals:
        set_param(logger, master, param_name, original_value, param_type)
        verify_param_change(logger, master, param_name, original_value)
    logger.log("[演示] 参数已恢复为修改前的值")


def main():
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    try:
        modify_specs = [] if args.list_only else parse_modify_specs(args.modify)
    except ValueError as exc:
        print(f"[错误] {exc}")
        return 1

    with OutputLogger(output_path) as logger:
        logger.log(f"[输出] 日志文件: {output_path}")

        try:
            master = connect(logger, args.connection, args.timeout)
        except Exception as exc:
            logger.log(f"[错误] 连接失败: {exc}")
            logger.log(
                "[提示] 请确认：\n"
                "  1. 虚拟机中 SITL 正在运行\n"
                "  2. 启动命令包含 --out=udp:<WindowsIP>:14551\n"
                "  3. 本脚本主动连接飞控，无需 QGC"
            )
            return 1

        params = fetch_all_params(logger, master, args.fetch_timeout)
        if not params:
            logger.log("[错误] 未收到任何 PARAM_VALUE 消息")
            return 1

        log_param_table(logger, params)

        if args.list_only:
            logger.log("[结束] --list-only 模式，跳过参数修改演示")
        else:
            run_modify_demo(
                logger,
                master,
                params,
                modify_specs,
                keep_changes=args.keep_changes,
            )

        logger.log(f"[输出] 记录已保存至: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
