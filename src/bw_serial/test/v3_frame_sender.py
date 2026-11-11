#!/usr/bin/env python3
"""V3 帧发送/接收调试工具（不依赖 ROS，也不需要节点在跑）。

用途：
  1. 直接向底盘下发"指定内容"的控制帧（0x21 底盘 / 0x01 整机），
     用来验证某一组 vx/vy/omega/滑台高度 到底能不能让底盘动；
  2. 反向伪造底盘反馈帧（0x22 / 0x02），在没有硬件时调试 mantis_comm_node；
  3. 监听总线上节点发出的帧并解包打印。

帧格式（与 src/v3_frame_codec.cpp 一致）：
  55 AA | TYPE | LEN_L | LEN_H | PAYLOAD | CRC_L | CRC_H
  CRC16/MODBUS，覆盖 TYPE + LEN(2B) + PAYLOAD

示例：
  # 只看要发什么（不打开串口）
  python3 v3_frame_sender.py --dry-run --mode chassis --vx 0.2 --omega 0.1 --lift 1300

  # 真的发给底盘，发 100 帧、每 10ms 一帧，然后听 2 秒反馈
  python3 v3_frame_sender.py --port /dev/ttyACM0 --baud 4000000 \
      --mode chassis --ctrl 1 --vx 0.2 --omega 0.1 --lift 1300 \
      --count 100 --period-ms 10 --listen 2

  # 没有硬件时：自己开一个 pty，把路径打印出来，让节点连上来
  python3 v3_frame_sender.py --pty --mode chassis-feedback --flags 0x07 ... --count 500
"""

from __future__ import annotations

import argparse
import copy
import os
import pty
import select
import struct
import sys
import time

HEADER = b"\x55\xaa"
TYPE_CHASSIS_CMD = 0x21
TYPE_CHASSIS_FB = 0x22
TYPE_FULL_CMD = 0x01
TYPE_FULL_FB = 0x02

LEN_CHASSIS = 37
LEN_FULL = 249
LEN_FULL_FB = 248


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def wrap(frame_type: int, payload: bytes) -> bytes:
    head = bytes([frame_type, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    return HEADER + head + payload + struct.pack("<H", crc16_modbus(head + payload))


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


# --------------------------------------------------------------------------- 组帧


def build_chassis_command(args) -> bytes:
    payload = struct.pack(
        "<B9f",
        args.ctrl & 0xFF,
        args.vx,
        args.vy,
        args.omega,
        args.acc_x,
        args.acc_y,
        args.acc_omega,
        args.lift,
        args.lift_v,
        args.lift_a,
    )
    assert len(payload) == LEN_CHASSIS
    return wrap(TYPE_CHASSIS_CMD, payload)


def build_full_command(args) -> bytes:
    values: list[float] = [args.vx, args.vy, args.omega, args.acc_x, args.acc_y, args.acc_omega]
    values += [args.lift, args.lift_v]
    values += args.left_pos + args.right_pos          # 16 个关节位置
    values += [args.waist, args.head_yaw, args.head_pitch]
    values += [0.0] * 16                              # 关节最大速度
    values += [args.waist_max_vel, args.head_yaw_max_vel, args.head_pitch_max_vel]
    values += [0.0] * 16                              # 力矩
    payload = struct.pack("<B62f", args.ctrl & 0xFF, *values)
    assert len(payload) == LEN_FULL, len(payload)
    return wrap(TYPE_FULL_CMD, payload)


def build_chassis_feedback(args) -> bytes:
    payload = struct.pack(
        "<B9f",
        args.flags & 0xFF,
        args.vx,
        args.vy,
        args.omega,
        args.odom_x,
        args.odom_y,
        args.heading,
        args.heading_vel,
        args.lift,
        args.lift_v,
    )
    assert len(payload) == LEN_CHASSIS
    return wrap(TYPE_CHASSIS_FB, payload)


def build_full_feedback(args) -> bytes:
    values: list[float] = [args.vx, args.vy, args.omega, args.odom_x, args.odom_y]
    values += [args.heading, args.heading_vel, args.lift, args.lift_v]
    values += args.left_pos + args.right_pos
    values += [args.waist, args.head_yaw, args.head_pitch]
    values += [0.0] * 16          # 关节速度
    values += [0.0]                # waist 速度
    values += [0.0] * 16          # 力矩
    payload = bytes([args.flags & 0xFF, args.arm_flags & 0xFF, args.arm_flags & 0xFF, args.chassis_flags & 0xFF])
    payload += struct.pack("<61f", *values)
    assert len(payload) == LEN_FULL_FB, len(payload)
    return wrap(TYPE_FULL_FB, payload)


BUILDERS = {
    "chassis": build_chassis_command,
    "full": build_full_command,
    "chassis-feedback": build_chassis_feedback,
    "full-feedback": build_full_feedback,
}


# --------------------------------------------------------------------------- 解包


def describe(frame_type: int, payload: bytes) -> str:
    if frame_type in (TYPE_CHASSIS_CMD, TYPE_CHASSIS_FB) and len(payload) == LEN_CHASSIS:
        ctrl, *vals = struct.unpack("<B9f", payload)
        if frame_type == TYPE_CHASSIS_CMD:
            return (
                f"0x21 chassis cmd ctrl=0x{ctrl:02X} vx={vals[0]:.3f} vy={vals[1]:.3f} "
                f"omega={vals[2]:.3f} acc=({vals[3]:.3f}, {vals[4]:.3f}, {vals[5]:.3f}) "
                f"lift={vals[6]:.1f}mm/{vals[7]:.1f}mm/s a={vals[8]:.1f}"
            )
        return (
            f"0x22 chassis feedback flags=0x{ctrl:02X} vx={vals[0]:.3f} vy={vals[1]:.3f} "
            f"omega={vals[2]:.3f} odom=({vals[3]:.3f}, {vals[4]:.3f}) "
            f"heading={vals[5]:.3f} w={vals[6]:.3f} lift={vals[7]:.1f}mm/{vals[8]:.1f}mm/s"
        )
    if frame_type == TYPE_FULL_CMD and len(payload) == LEN_FULL:
        ctrl = payload[0]
        vals = struct.unpack("<62f", payload[1:])
        return (
            f"0x01 full cmd ctrl=0x{ctrl:02X} vx={vals[0]:.3f} omega={vals[2]:.3f} "
            f"lift={vals[6]:.1f} waist={vals[24]:.3f}"
        )
    if frame_type == TYPE_FULL_FB and len(payload) == LEN_FULL_FB:
        flags = payload[0]
        vals = struct.unpack("<61f", payload[4:])
        return (
            f"0x02 full feedback status=0x{flags:02X} vx={vals[0]:.3f} omega={vals[2]:.3f} "
            f"lift={vals[7]:.1f} waist={vals[25]:.3f}"
        )
    return f"type=0x{frame_type:02X} len={len(payload)}（未识别）"


def iter_frames(buffer: bytearray):
    while True:
        index = buffer.find(HEADER)
        if index < 0 or len(buffer) < index + 5:
            return
        if index > 0:
            del buffer[:index]
        frame_type = buffer[2]
        payload_length = buffer[3] | (buffer[4] << 8)
        total = 5 + payload_length + 2
        if len(buffer) < total:
            return
        payload = bytes(buffer[5 : 5 + payload_length])
        crc_ok = crc16_modbus(bytes(buffer[2:5]) + payload) == (
            buffer[total - 2] | (buffer[total - 1] << 8)
        )
        del buffer[:total]
        yield frame_type, payload, crc_ok


# --------------------------------------------------------------------------- 收发


def open_serial(path: str, baud: int) -> int:
    import termios

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def parse_floats(text: str, count: int, default: float) -> list[float]:
    if not text:
        return [default] * count
    parts = [p for p in text.replace(" ", "").split(",") if p]
    values = [float(p) for p in parts]
    if len(values) != count:
        raise SystemExit(f"需要 {count} 个数，实际给了 {len(values)} 个：{text}")
    return values


def drain(fd: int, buffer: bytearray, rounds: int = 4) -> None:
    """非阻塞读几次，最多 rounds 轮，避免对端持续回帧时卡死。"""
    for _ in range(rounds):
        if not select.select([fd], [], [], 0)[0]:
            return
        chunk = os.read(fd, 4096)
        if not chunk:
            return
        buffer.extend(chunk)


def build_zero_frame(args) -> bytes:
    """同一种帧结构，速度清零（滑台目标高度保持不变）。"""
    stopped = copy.copy(args)
    stopped.vx = 0.0
    stopped.vy = 0.0
    stopped.omega = 0.0
    return BUILDERS[args.mode](stopped)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="直接发送/接收 V3 控制帧与反馈帧（自带 CRC，不依赖 ROS）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="串口设备，默认 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=4000000, help="波特率，默认 4000000")
    parser.add_argument("--pty", action="store_true", help="不开串口，自己创建 pty 并打印从设备路径")
    parser.add_argument("--pty-file", default="", help="把 pty 路径写进这个文件（便于脚本读取）")
    parser.add_argument("--mode", choices=sorted(BUILDERS), default="chassis", help="要发送的帧类型")
    parser.add_argument("--ctrl", default=0x01, help="控制帧 Byte0 软件开关位，默认 0x01")
    parser.add_argument("--flags", default=0x07, help="反馈帧状态位，默认 0x07")
    parser.add_argument("--arm-flags", default=0x00, help="整机反馈左右臂状态位")
    parser.add_argument("--chassis-flags", default=0x00, help="整机反馈底盘状态位")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--omega", type=float, default=0.0)
    parser.add_argument("--acc-x", type=float, default=1.5)
    parser.add_argument("--acc-y", type=float, default=1.5)
    parser.add_argument("--acc-omega", type=float, default=1.5)
    parser.add_argument("--lift", type=float, default=1200.0, help="滑台高度 mm")
    parser.add_argument("--lift-v", type=float, default=0.0, help="滑台速度 mm/s（反馈用）")
    parser.add_argument("--lift-a", type=float, default=1000.0, help="滑台加速度 mm/s^2")
    parser.add_argument("--odom-x", type=float, default=0.0)
    parser.add_argument("--odom-y", type=float, default=0.0)
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--heading-vel", type=float, default=0.0)
    parser.add_argument("--waist", type=float, default=0.0)
    parser.add_argument("--head-yaw", type=float, default=0.0)
    parser.add_argument("--head-pitch", type=float, default=0.0)
    parser.add_argument("--waist-max-vel", type=float, default=0.0)
    parser.add_argument("--head-yaw-max-vel", type=float, default=0.0)
    parser.add_argument("--head-pitch-max-vel", type=float, default=0.0)
    parser.add_argument("--left-pos", default="", help="整机：左臂 8 个位置，逗号分隔")
    parser.add_argument("--right-pos", default="", help="整机：右臂 8 个位置，逗号分隔")
    parser.add_argument("--count", type=int, default=1, help="发送帧数，默认 1（反馈模式默认 1）")
    parser.add_argument("--period-ms", type=float, default=100.0, help="发送间隔 ms")
    parser.add_argument("--listen", type=float, default=0.0, help="发完后监听多少秒并解包打印")
    parser.add_argument("--delay", type=float, default=0.0, help="开始发送前等待多少秒")
    parser.add_argument("--stop-frames", type=int, default=10, help="退出前补发多少帧零速度，默认 10")
    parser.add_argument(
        "--stop-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="退出前（含 Ctrl+C）自动补发零速度帧，默认开启；用 --no-stop-on-exit 关闭",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印十六进制帧，不打开任何设备")
    args = parser.parse_args()

    args.ctrl = int(str(args.ctrl), 0)
    args.flags = int(str(args.flags), 0)
    args.arm_flags = int(str(args.arm_flags), 0)
    args.chassis_flags = int(str(args.chassis_flags), 0)
    args.left_pos = parse_floats(args.left_pos, 8, 0.0)
    args.right_pos = parse_floats(args.right_pos, 8, 0.0)

    frame = BUILDERS[args.mode](args)
    print(f"[frame] {args.mode}: {len(frame)} 字节")
    print(f"[hex]   {hexdump(frame)}")

    if args.dry_run:
        for frame_type, payload, ok in iter_frames(bytearray(frame)):
            print(f"[self]  crc_ok={ok} {describe(frame_type, payload)}")
        return 0

    if args.pty:
        master, slave = pty.openpty()
        slave_name = os.ttyname(slave)
        print(f"[pty]   从设备：{slave_name}")
        print(f"[pty]   另开终端：ros2 run bw_serial mantis_comm_node --ros-args "
              f"-r __ns:=$SN -p port_name:={slave_name} -p protocol_version:=v3_chassis ...")
        if args.pty_file:
            with open(args.pty_file, "w", encoding="utf-8") as handle:
                handle.write(slave_name)
        fd = master
    else:
        fd = open_serial(args.port, args.baud)
        print(f"[port]  已打开 {args.port} @ {args.baud}")

    if args.delay > 0:
        print(f"[wait]  {args.delay:.1f}s 后开始发送")
        time.sleep(args.delay)

    buffer = bytearray()
    received = 0
    interrupted = False
    try:
        for index in range(args.count):
            os.write(fd, frame)
            if args.period_ms > 0 and index + 1 < args.count:
                time.sleep(args.period_ms / 1000.0)
            if args.listen <= 0 and args.period_ms > 0:
                # 边发边收，避免对端发送缓冲被写满（有上限，不会卡死）
                drain(fd, buffer)
            if index == 0:
                print(f"[send]  {args.count} 帧，间隔 {args.period_ms}ms")
                if args.count == 1:
                    print("[hint]  只发 1 帧是 --count 的默认值；退出时会自动补发零速度帧")

        if args.listen > 0:
            deadline = time.time() + args.listen
            while time.time() < deadline:
                if not select.select([fd], [], [], 0.05)[0]:
                    continue
                chunk = os.read(fd, 4096)
                if not chunk:
                    continue
                buffer.extend(chunk)
                for frame_type, payload, ok in iter_frames(buffer):
                    received += 1
                    mark = "OK " if ok else "CRC!"
                    print(f"[recv]  {mark} {describe(frame_type, payload)}")
            print(f"[done]  收到 {received} 帧")
    except KeyboardInterrupt:
        interrupted = True
        print("\n[stop]  收到 Ctrl+C")
    finally:
        if args.stop_on_exit and args.count > 0 and args.mode in ("chassis", "full"):
            zero = build_zero_frame(args)
            sent = 0
            for _ in range(max(0, args.stop_frames)):
                try:
                    os.write(fd, zero)
                except OSError:
                    break
                sent += 1
                time.sleep(0.02)
            print(f"[stop]  已补发 {sent} 帧零速度（vx=vy=omega=0，lift={args.lift:.0f}mm 不变）")
        os.close(fd)
    return 130 if interrupted else 0


if __name__ == "__main__":
    sys.exit(main())
