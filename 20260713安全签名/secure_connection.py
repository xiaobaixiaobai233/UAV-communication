#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
secure_connection.py - MAVLink 2 安全签名连接与验证

任务要求（安全签名阶段）：
  1. 在创建连接时配置 signing_key（与飞控 MAVProxy 中 setup 的密钥一致）
  2. 验证签名功能：未配钥 / 错误密钥时，控制类请求应被飞控拒绝
  3. 对比实验并输出日志，供实验报告使用

飞控侧启用签名（Ubuntu VM · MAVProxy 控制台）：
  module load signing
  signing setup my_sitl_mavlink2_signing_key!!!!

  实验结束后恢复：
  signing remove my_sitl_mavlink2_signing_key!!!!

SITL 启动示例：
  python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter --console \\
      --out=udp:<Windows主机IP>:14551

用法：
  python secure_connection.py --mode signed
  python secure_connection.py --mode unsigned
  python secure_connection.py --mode wrong-key
  python secure_connection.py --mode all
  python secure_connection.py --mode signed --key my_sitl_mavlink2_signing_key!!!!
"""

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

# MAVLink 2 签名仅支持 MAVLink 2 协议
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

DEFAULT_LISTEN_PORT = 14551
DEFAULT_SECRET_KEY = b"my_sitl_mavlink2_signing_key!!!!"
DEFAULT_WRONG_KEY = b"wrong_sitl_mavlink2_sign_key!!!!"
PARAM_FETCH_TIMEOUT_SEC = 12.0
TELEMETRY_SAMPLE_SEC = 3.0

MODE_LABELS = {
    "signed": "实验 A · 正确密钥",
    "unsigned": "实验 B · 未配置密钥",
    "wrong-key": "实验 C · 错误密钥",
}


class OutputLogger:
    """日志记录器：stderr 显示 + 文件写入。"""

    def __init__(self, filepath: Path):
        self.filepath = filepath.resolve()
        self._file = self.filepath.open("w", encoding="utf-8", newline="\n")
        self._write_header()

    def _write_header(self):
        lines = [
            "MAVLink 2 安全签名验证记录",
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
        description="MAVLink 2 安全签名：连接配置与三组对比验证"
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
        "--fetch-timeout",
        type=float,
        default=PARAM_FETCH_TIMEOUT_SEC,
        help=f"参数拉取等待超时（秒），默认 {PARAM_FETCH_TIMEOUT_SEC:.0f}",
    )
    parser.add_argument(
        "--mode",
        choices=["signed", "unsigned", "wrong-key", "all"],
        default="all",
        help="验证模式：signed / unsigned / wrong-key / all（默认跑三组）",
    )
    parser.add_argument(
        "--key",
        default=DEFAULT_SECRET_KEY.decode("ascii"),
        help="与飞控一致的 32 字节签名密钥",
    )
    parser.add_argument(
        "--wrong-key",
        default=DEFAULT_WRONG_KEY.decode("ascii"),
        help="错误密钥（wrong-key 模式使用），默认 32 字节测试串",
    )
    parser.add_argument(
        "--output",
        default="",
        help="日志文件路径；默认自动生成 signing_output_时间戳.txt",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"signing_output_{stamp}.txt"


def normalize_secret_key(key_text: str, label: str) -> bytes:
    key_bytes = key_text.encode("utf-8")
    if len(key_bytes) != 32:
        raise ValueError(
            f"{label} 必须为 32 字节，当前为 {len(key_bytes)} 字节: {key_text!r}"
        )
    return key_bytes


# 飞控下行遥测通常不带签名；启用 signing 后通过 msgId 白名单接受未签名入站消息
UNSIGNED_INBOUND_MSG_IDS = frozenset(
    {
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        mavutil.mavlink.MAVLINK_MSG_ID_PARAM_VALUE,
        mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK,
        mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS,
    }
)


def allow_unsigned_telemetry(_mav, msg_id: int) -> bool:
    """pymavlink 回调签名: (mavlink_instance, msgId) -> bool"""
    return msg_id in UNSIGNED_INBOUND_MSG_IDS


def connect(
    logger: OutputLogger,
    connection_string: str,
    timeout: float,
    secret_key: Optional[bytes],
    mode_label: str,
):
    logger.log(f"[连接] {mode_label}")
    logger.log(f"[连接] 正在监听: {connection_string}")

    master = mavutil.mavlink_connection(connection_string)

    # 先等心跳，再配置签名：飞控 HEARTBEAT 通常未签名，顺序反了会收不到
    logger.log("[连接] 等待 HEARTBEAT（此时尚未配置出站签名）...")
    master.wait_heartbeat(timeout=timeout)
    logger.log(
        f"[连接] 已收到心跳 | target_system={master.target_system}, "
        f"target_component={master.target_component}"
    )

    if secret_key is None:
        logger.log("[签名] 未调用 setup_signing()（出站消息不带签名）")
    else:
        master.setup_signing(
            secret_key,
            sign_outgoing=True,
            allow_unsigned_callback=allow_unsigned_telemetry,
        )
        logger.log(
            f"[签名] 已 setup_signing，密钥长度={len(secret_key)} 字节；"
            "入站遥测仍接受未签名 HEARTBEAT/PARAM_VALUE 等"
        )

    return master


def sample_telemetry(logger: OutputLogger, master, duration: float) -> dict:
    """被动接收遥测，验证下行数据在签名启用时仍可到达。"""
    counts = {"HEARTBEAT": 0, "GLOBAL_POSITION_INT": 0, "ATTITUDE": 0, "other": 0}
    deadline = time.time() + duration
    logger.log(f"[遥测] 被动监听 {duration:.0f}s，统计下行消息...")

    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type in counts:
            counts[msg_type] += 1
        else:
            counts["other"] += 1

    logger.log(
        "[遥测] "
        f"HEARTBEAT={counts['HEARTBEAT']}, "
        f"GLOBAL_POSITION_INT={counts['GLOBAL_POSITION_INT']}, "
        f"ATTITUDE={counts['ATTITUDE']}, "
        f"other={counts['other']}"
    )
    return counts


def test_param_request(logger: OutputLogger, master, timeout: float) -> dict:
    """
    发送 PARAM_REQUEST_LIST，统计 PARAM_VALUE 响应数量。

    飞控启用签名后，仅接受带正确签名的参数请求。
    """
    logger.log("[验证] 发送 param_request_list() 请求全部参数...")
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component,
    )

    params = {}
    expected_count = None
    start = time.time()

    while True:
        if time.time() - start > timeout:
            break

        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if expected_count is not None and len(params) >= expected_count:
                break
            continue

        if isinstance(msg.param_id, bytes):
            name = msg.param_id.decode("utf-8", errors="replace").rstrip("\x00")
        else:
            name = str(msg.param_id).rstrip("\x00")

        params[name] = msg.param_value
        if expected_count is None:
            expected_count = msg.param_count
            logger.log(f"[验证] 飞控报告参数总数: {expected_count}")

        if expected_count and len(params) >= expected_count:
            break

    received = len(params)
    complete = expected_count is not None and received >= expected_count
    logger.log(f"[验证] 实际收到 PARAM_VALUE: {received} 条")

    if complete:
        logger.log("[验证] 参数列表拉取完整 — 签名验证通过（或飞控未启用签名）")
    elif received > 0:
        logger.log("[验证] 收到部分参数 — 可能超时或签名不匹配导致中断")
    else:
        logger.log("[验证] 未收到任何 PARAM_VALUE — 请求很可能被飞控拒绝")

    return {
        "expected_count": expected_count,
        "received_count": received,
        "complete": complete,
    }


def test_command_request(logger: OutputLogger, master, timeout: float = 5.0) -> dict:
    """
    发送 harmless 的 SET_MESSAGE_INTERVAL 命令，检查 COMMAND_ACK。

    比参数请求更直接地验证「控制命令是否被接受」。
    """
    logger.log("[验证] 发送 MAV_CMD_SET_MESSAGE_INTERVAL（HEARTBEAT @ 1Hz）...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        1_000_000,
        0,
        0,
        0,
        0,
        0,
    )

    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if ack is None:
        logger.log(f"[验证] 未收到 COMMAND_ACK（{timeout:.0f}s 超时）— 命令可能被拒绝")
        return {"ack_received": False, "result": None, "accepted": False}

    result_names = {
        0: "ACCEPTED",
        1: "TEMPORARILY_REJECTED",
        2: "DENIED",
        3: "UNSUPPORTED",
        4: "FAILED",
        5: "IN_PROGRESS",
        6: "CANCELLED",
    }
    result_name = result_names.get(ack.result, str(ack.result))
    accepted = ack.result in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )
    logger.log(
        f"[验证] COMMAND_ACK | command={ack.command}, "
        f"result={ack.result} ({result_name})"
    )
    return {"ack_received": True, "result": ack.result, "accepted": accepted}


def summarize_experiment(logger: OutputLogger, mode: str, telemetry: dict, param_result: dict, cmd_result: dict):
    label = MODE_LABELS.get(mode, mode)
    logger.log("-" * 72)
    logger.log(f"[结论] {label}")

    telemetry_ok = telemetry["HEARTBEAT"] > 0 or telemetry["GLOBAL_POSITION_INT"] > 0
    param_ok = param_result["complete"]
    cmd_ok = cmd_result["accepted"]

    logger.log(f"  下行遥测可达: {'是' if telemetry_ok else '否'}")
    logger.log(f"  参数请求成功: {'是' if param_ok else '否'} ({param_result['received_count']} 条)")
    logger.log(
        f"  控制命令接受: {'是' if cmd_ok else '否'}"
        + (f" (ACK result={cmd_result['result']})" if cmd_result["ack_received"] else " (无 ACK)")
    )

    if mode == "signed":
        expected = param_ok and cmd_ok
        logger.log(f"  预期（飞控已启用签名）: 参数与控制均应成功 → {'符合预期' if expected else '不符合预期'}")
    else:
        expected = not param_ok or not cmd_ok
        logger.log(
            f"  预期（飞控已启用签名）: 参数或控制应被拒绝 → "
            f"{'符合预期' if expected else '不符合预期（飞控可能未启用签名）'}"
        )

    logger.log("-" * 72)
    return param_ok and cmd_ok


def resolve_signing_key(mode: str, secret_key: bytes, wrong_key: bytes) -> Optional[bytes]:
    if mode == "signed":
        return secret_key
    if mode == "wrong-key":
        return wrong_key
    return None


def run_single_experiment(
    logger: OutputLogger,
    connection_string: str,
    timeout: float,
    fetch_timeout: float,
    mode: str,
    secret_key: bytes,
    wrong_key: bytes,
) -> bool:
    signing_key = resolve_signing_key(mode, secret_key, wrong_key)
    label = MODE_LABELS.get(mode, mode)

    logger.log("")
    logger.log("=" * 72)
    logger.log(label)
    logger.log("=" * 72)

    try:
        master = connect(logger, connection_string, timeout, signing_key, label)
    except Exception as exc:
        logger.log(f"[错误] 连接失败: {exc}")
        logger.log(
            "[提示] 请确认：\n"
            "  1. SITL 启动命令包含 --out=udp:<Windows主机IP>:14551\n"
            "     （不要用 127.0.0.1，那是虚拟机本机，Windows 收不到）\n"
            "  2. Windows 上用 ipconfig 查看 IPv4 地址（桥接网卡）\n"
            "  3. 若已启用签名，signed 模式密钥需与 MAVProxy 一致"
        )
        return False

    telemetry = sample_telemetry(logger, master, TELEMETRY_SAMPLE_SEC)
    param_result = test_param_request(logger, master, fetch_timeout)
    cmd_result = test_command_request(logger, master)
    return summarize_experiment(logger, mode, telemetry, param_result, cmd_result)


def main():
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    try:
        secret_key = normalize_secret_key(args.key, "签名密钥 (--key)")
        wrong_key = normalize_secret_key(args.wrong_key, "错误密钥 (--wrong-key)")
    except ValueError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1

    modes = ["signed", "unsigned", "wrong-key"] if args.mode == "all" else [args.mode]

    with OutputLogger(output_path) as logger:
        logger.log(f"[输出] 日志文件: {output_path}")
        logger.log("[说明] 本脚本验证 MAVLink 2 消息签名。")
        logger.log(
            "[说明] 飞控侧请先在 MAVProxy 执行:\n"
            "  module load signing\n"
            f"  signing setup {args.key}"
        )
        logger.log(
            "[说明] unsigned / wrong-key 模式下，若飞控未启用签名，"
            "三组实验可能均成功——此时请先配置飞控签名后再测。"
        )

        results = {}
        for mode in modes:
            results[mode] = run_single_experiment(
                logger,
                args.connection,
                args.timeout,
                args.fetch_timeout,
                mode,
                secret_key,
                wrong_key,
            )
            if len(modes) > 1 and mode != modes[-1]:
                logger.log("[等待] 下一组实验前暂停 2 秒...")
                time.sleep(2.0)

        if len(modes) > 1:
            logger.log("")
            logger.log("=" * 72)
            logger.log("[总结] 三组实验汇总")
            logger.log("=" * 72)
            for mode in modes:
                ok = results[mode]
                logger.log(f"  {MODE_LABELS.get(mode, mode)}: {'通过' if ok else '未通过'}")
            logger.log("")
            logger.log(
                "理想结果（飞控已启用签名）: "
                "signed=通过, unsigned=未通过, wrong-key=未通过"
            )

        logger.log(f"[输出] 记录已保存至: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
