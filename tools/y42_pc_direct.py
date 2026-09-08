#!/usr/bin/env python3
"""Direct PC -> USB-RS485 controller for the Y42/EMM motor bus.

This tool deliberately does not implement cable-length calibration or XY
trajectory control.  It is for bus discovery, motor-ID setup, telemetry and
low-speed single-motor tests before the mechanical calibration is known.

The protocol bytes are taken from Y42_ESP32S3_Debugger/Y42_ESP32S3_Debugger.ino
in this repository.  The default command set targets the Y42 X firmware.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - friendly CLI error
    print("缺少 pyserial，请先运行: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2) from exc


DEFAULT_BAUD = 115200
FRAME_END = 0x6B
MIN_ID = 1
MAX_ID = 255
OPERATING_CURRENT_MAX_MA = 2000

# The web controller may install a best-effort traffic recorder.  Keeping this
# hook optional preserves the standalone CLI and, critically, a recorder fault
# can never interrupt a motor command or its safety reply.
_TRAFFIC_CALLBACK = None


def set_traffic_callback(callback) -> None:
    global _TRAFFIC_CALLBACK
    _TRAFFIC_CALLBACK = callback


def _emit_traffic(record: dict) -> None:
    callback = _TRAFFIC_CALLBACK
    if callback is None:
        return
    try:
        callback(record)
    except Exception:
        return


def hex_bytes(data: bytes | bytearray | Iterable[int]) -> str:
    return " ".join(f"{int(value) & 0xFF:02X}" for value in data)


def u16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"16 位数值超范围: {value}")
    return bytes(((value >> 8) & 0xFF, value & 0xFF))


def u32(value: int) -> bytes:
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"32 位数值超范围: {value}")
    return bytes(
        (
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        )
    )


def frame(*values: int) -> bytes:
    return bytes(values)


def check_id(value: int, label: str = "ID") -> int:
    if not MIN_ID <= value <= MAX_ID:
        raise ValueError(f"{label} 必须在 {MIN_ID}..{MAX_ID} 之间: {value}")
    return value


def list_serial_ports() -> list[object]:
    return sorted(list_ports.comports(), key=lambda port: str(port.device))


def print_serial_ports() -> int:
    ports = list_serial_ports()
    if not ports:
        print("没有发现串口。请插入 USB-RS485 转换器并检查驱动。")
        return 1
    for port in ports:
        description = str(port.description or "").strip()
        suffix = f" - {description}" if description else ""
        print(f"{port.device}{suffix}")
    return 0


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = list_serial_ports()
    if len(ports) == 1:
        return str(ports[0].device)
    if not ports:
        raise RuntimeError("没有发现串口，请先运行 ports 或插入 USB-RS485 转换器")
    names = ", ".join(str(port.device) for port in ports)
    raise RuntimeError(f"发现多个串口 ({names})，请使用 --port COMx 指定 USB-RS485 串口")


def open_serial(port: str, baud: int, *, direction_mode: str = "auto") -> serial.Serial:
    if direction_mode not in {"auto", "rts-high", "rts-low"}:
        raise ValueError("RS485 收发方向必须是 auto、rts-high 或 rts-low")
    connection = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        # Use short polling reads.  The old 10 ms read timeout was paid again
        # after a complete reply had already arrived, inflating each status
        # round-trip and starving the 100 Hz trajectory clock.
        timeout=0.001,
        write_timeout=2.0,
        rtscts=False,
        dsrdtr=False,
    )
    # Do not toggle modem-control lines; some USB-RS485 adapters use them for
    # direction control or reset behavior.
    try:
        connection.dtr = False
        # Leave RTS untouched in auto mode: many USB-RS485 adapters use their
        # own transceiver direction control, while bare MAX485 boards may wire
        # DE/RE to RTS explicitly.
        if direction_mode == "rts-high":
            connection.rts = True
        elif direction_mode == "rts-low":
            connection.rts = False
    except (AttributeError, serial.SerialException):
        pass
    connection._y42_direction_mode = direction_mode
    connection.reset_input_buffer()
    connection.reset_output_buffer()
    return connection


def transact(
    connection: serial.Serial,
    tx: bytes,
    *,
    first_byte_timeout: float = 0.06,
    inter_byte_quiet: float = 0.004,
    max_rx: int = 128,
    deadline: float | None = None,
) -> bytes:
    """Send a query with optional total budget, preserving even failed traffic."""

    transaction_started_ns = time.perf_counter_ns()
    direction_mode = getattr(connection, "_y42_direction_mode", "auto")
    received = bytearray()
    failure = None
    previous_write_timeout = connection.write_timeout if deadline is not None else None
    previous_read_timeout = connection.timeout if deadline is not None else None

    def remaining_budget() -> float:
        remaining = float(deadline) - time.perf_counter()
        if remaining <= 0.0:
            raise TimeoutError("串口只读检查超过时间预算")
        return remaining

    try:
        if deadline is not None:
            remaining_budget()
        if direction_mode == "rts-high":
            connection.rts = True
        elif direction_mode == "rts-low":
            connection.rts = False
        connection.reset_input_buffer()
        if deadline is not None:
            remaining = remaining_budget()
            bounded_write_timeout = min(
                remaining,
                float(previous_write_timeout) if previous_write_timeout is not None else remaining,
            )
            if connection.write_timeout != bounded_write_timeout:
                connection.write_timeout = bounded_write_timeout
        connection.write(tx)
        # Windows flush() can block indefinitely on an unplugged driver.  A
        # bounded query waits for the reply instead of explicitly draining TX.
        if deadline is None:
            connection.flush()
        if direction_mode in {"rts-high", "rts-low"}:
            baudrate = float(getattr(connection, "baudrate", DEFAULT_BAUD) or DEFAULT_BAUD)
            delay = max(0.001, len(tx) * 10.0 / baudrate)
            time.sleep(delay if deadline is None else min(delay, remaining_budget()))
            connection.rts = direction_mode == "rts-low"

        started = time.monotonic()
        last_byte = started
        while len(received) < max_rx:
            if deadline is not None:
                remaining = remaining_budget()
                bounded_read_timeout = min(
                    remaining,
                    float(previous_read_timeout) if previous_read_timeout is not None else 0.001,
                )
                # PySerial on Windows reconfigures the port on every setter,
                # including unchanged values. Keep the deadline without that IO.
                if connection.timeout != bounded_read_timeout:
                    connection.timeout = bounded_read_timeout
            waiting = int(getattr(connection, "in_waiting", 0) or 0)
            chunk = connection.read(min(max_rx - len(received), waiting)) if waiting else connection.read(1)
            if chunk:
                received.extend(chunk)
                last_byte = time.monotonic()
                continue
            now = time.monotonic()
            if received and now - last_byte >= inter_byte_quiet:
                break
            if not received and now - started >= first_byte_timeout:
                break
        return bytes(received)
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if deadline is not None:
            try:
                if connection.write_timeout != previous_write_timeout:
                    connection.write_timeout = previous_write_timeout
                if connection.timeout != previous_read_timeout:
                    connection.timeout = previous_read_timeout
            except (AttributeError, OSError, serial.SerialException):
                pass
        _emit_traffic(
            {
                "channel": "synchronous_transaction",
                "port": str(getattr(connection, "port", "") or ""),
                "baud": int(getattr(connection, "baudrate", DEFAULT_BAUD) or DEFAULT_BAUD),
                "direction_mode": str(direction_mode),
                "tx_hex": tx.hex(),
                "tx_bytes": len(tx),
                "rx_hex": bytes(received).hex(),
                "rx_bytes": len(received),
                "started_monotonic_ns": transaction_started_ns,
                "completed_monotonic_ns": time.perf_counter_ns(),
                **({"error": failure} if failure else {}),
            }
        )


def valid_reply(reply: bytes, address: int, command: int, minimum: int = 4) -> bool:
    return (
        len(reply) >= minimum
        and reply[0] == address
        and reply[1] == command
        and reply[-1] == FRAME_END
    )


def command_accepted(reply: bytes, address: int, command: int) -> bool:
    # Some USB-RS485 adapters return the local TX echo before the motor ACK.
    # For short query frames this is usually harmless, but a long C5/FD frame
    # would make reply[2] point at the echoed direction byte.  Accept the
    # standard ACK either as the whole reply or as a complete frame contained
    # in an echo-prefixed reply.
    if valid_reply(reply, address, command, 4) and reply[2] == 0x02:
        return True
    return bytes((address, command, 0x02, FRAME_END)) in reply


def status_query(connection: serial.Serial, address: int) -> bytes:
    return transact(connection, frame(address, 0x3A, FRAME_END))


def decode_flags(flags: int) -> str:
    names = []
    if flags & 0x01:
        names.append("enabled")
    if flags & 0x02:
        names.append("reached")
    if flags & 0x04:
        names.append("stall")
    if flags & 0x08:
        names.append("protected")
    if flags & 0x10:
        names.append("left-limit")
    if flags & 0x20:
        names.append("right-limit")
    return ",".join(names) if names else "none"


def read_u16(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def read_u32(data: bytes, offset: int) -> int:
    return (
        (data[offset] << 24)
        | (data[offset + 1] << 16)
        | (data[offset + 2] << 8)
        | data[offset + 3]
    )


@dataclass
class Telemetry:
    address: int
    voltage: float | None = None
    bus_current: int | None = None
    phase_current: int | None = None
    rpm: float | None = None
    position: float | None = None
    temperature: int | None = None
    flags: int | None = None


def read_telemetry(
    connection: serial.Serial,
    address: int,
    *,
    first_byte_timeout: float = 0.06,
    inter_byte_quiet: float = 0.004,
    deadline: float | None = None,
) -> Telemetry:
    telemetry = Telemetry(address)
    queries = (
        ("voltage", 0x24),
        ("bus_current", 0x26),
        ("phase_current", 0x27),
        ("rpm", 0x35),
        ("position", 0x36),
        ("temperature", 0x39),
        ("flags", 0x3A),
    )
    for name, command in queries:
        reply = transact(
            connection,
            frame(address, command, FRAME_END),
            first_byte_timeout=first_byte_timeout,
            inter_byte_quiet=inter_byte_quiet,
            **({"deadline": deadline} if deadline is not None else {}),
        )
        if command == 0x24 and valid_reply(reply, address, command, 5):
            telemetry.voltage = read_u16(reply, 2) / 1000.0
        elif command == 0x26 and valid_reply(reply, address, command, 5):
            telemetry.bus_current = read_u16(reply, 2)
        elif command == 0x27 and valid_reply(reply, address, command, 5):
            telemetry.phase_current = read_u16(reply, 2)
        elif command == 0x35 and valid_reply(reply, address, command, 6):
            value = read_u16(reply, 3) / 10.0
            telemetry.rpm = -value if reply[2] else value
        elif command == 0x36 and valid_reply(reply, address, command, 8):
            value = read_u32(reply, 3) / 10.0
            telemetry.position = -value if reply[2] else value
        elif command == 0x39 and valid_reply(reply, address, command, 5):
            telemetry.temperature = reply[3]
        elif command == 0x3A and valid_reply(reply, address, command, 4):
            telemetry.flags = reply[2]
    return telemetry


def print_telemetry(telemetry: Telemetry) -> None:
    def show(value: object, suffix: str = "") -> str:
        return "--" if value is None else f"{value}{suffix}"

    flags = "--" if telemetry.flags is None else f"0x{telemetry.flags:02X} ({decode_flags(telemetry.flags)})"
    print(f"ID {telemetry.address}")
    print(f"  voltage={show(telemetry.voltage, ' V')}")
    print(f"  bus_current={show(telemetry.bus_current, ' mA')}")
    print(f"  phase_current={show(telemetry.phase_current, ' mA')}")
    print(f"  rpm={show(telemetry.rpm)}")
    print(f"  position={show(telemetry.position, ' deg')}")
    print(f"  temperature={show(telemetry.temperature, ' C')}")
    print(f"  flags={flags}")


def confirm(message: str, yes: bool) -> None:
    if yes:
        return
    answer = input(f"{message}\n请输入 YES 继续，其他内容取消: ").strip()
    if answer != "YES":
        raise RuntimeError("用户取消")


def set_motor_id(connection: serial.Serial, old_id: int, new_id: int) -> bytes:
    check_id(old_id, "旧 ID")
    check_id(new_id, "新 ID")
    if old_id == new_id:
        raise ValueError("旧 ID 和新 ID 不能相同")
    # EMM/Y42 parameter write used by the repository's setid implementation.
    return transact(connection, frame(old_id, 0xAE, 0x4B, 0x01, new_id, FRAME_END), first_byte_timeout=0.3)


def position_frame(address: int, direction: int, rpm: float, acceleration: int, degrees: float, mode: int) -> bytes:
    """Build an X-firmware trapezoid position frame.

    The angle field is always a movement amount in 0.1° units.  Mode 0 is
    relative to the previous target, mode 1 is absolute from the coordinate
    zero, and mode 2 is relative to the current real-time position.
    """
    if direction not in (0, 1):
        raise ValueError("方向必须是 0 或 1")
    if not 0.0 < rpm <= 3000.0:
        raise ValueError("RPM 必须在 (0, 3000] 之间")
    if not 0 <= acceleration <= 65535:
        raise ValueError("X 固件加速度必须在 0..65535")
    if not 0 <= mode <= 2:
        raise ValueError("位置模式必须是 0、1 或 2")
    angle10 = round(abs(degrees) * 10.0)
    return (
        frame(address, 0xFD, direction)
        + u16(acceleration)
        + u16(acceleration)
        + u16(round(rpm * 10.0))
        + u32(angle10)
        + frame(mode, 0x00, FRAME_END)
    )


def torque_frame(address: int, direction: int, current: int, slope: int, max_rpm: float) -> bytes:
    if direction not in (0, 1):
        raise ValueError("方向必须是 0 或 1")
    if not 0 <= current <= OPERATING_CURRENT_MAX_MA:
        raise ValueError(f"电流必须在 0..{OPERATING_CURRENT_MAX_MA} mA")
    if not 0 <= slope <= 65535:
        raise ValueError("斜率必须在 0..65535")
    if not 0.1 <= max_rpm <= 3000.0:
        raise ValueError("最大 RPM 必须在 0.1..3000")
    return (
        frame(address, 0xC5, direction)
        + u16(slope)
        + u16(current)
        + frame(0x00)
        + u16(round(max_rpm * 10.0))
        + frame(FRAME_END)
    )


def speed_frame(address: int, direction: int, rpm: float, acceleration: int) -> bytes:
    if direction not in (0, 1):
        raise ValueError("方向必须是 0 或 1")
    if not 0.0 <= rpm <= 3000.0:
        raise ValueError("RPM 必须在 0..3000")
    if not 0 <= acceleration <= 65535:
        raise ValueError("加速度必须在 0..65535")
    return (
        frame(address, 0xF6, direction)
        + u16(acceleration)
        + u16(round(rpm * 10.0))
        + frame(0x00, FRAME_END)
    )


def parse_hex_frame(text: str) -> bytes:
    tokens = text.replace(",", " ").split()
    if not 3 <= len(tokens) <= 128:
        raise ValueError("原始帧长度必须是 3..128 字节")
    values = []
    for token in tokens:
        try:
            value = int(token, 16)
        except ValueError as exc:
            raise ValueError(f"不是十六进制字节: {token}") from exc
        if not 0 <= value <= 0xFF:
            raise ValueError(f"字节超范围: {token}")
        values.append(value)
    return bytes(values)


def scan(connection: serial.Serial, start: int, end: int, verbose: bool) -> list[int]:
    found: list[int] = []
    ambiguous: list[int] = []
    for address in range(start, end + 1):
        reply = status_query(connection, address)
        if valid_reply(reply, address, 0x3A, 4):
            found.append(address)
            print(f"发现 ID {address}: flags=0x{reply[2]:02X} ({decode_flags(reply[2])})")
        elif reply:
            ambiguous.append(address)
            if verbose:
                print(f"ID {address} 返回异常帧: {hex_bytes(reply)}")
        elif verbose:
            print(f"ID {address}: timeout")
    if ambiguous:
        print(f"异常/碰撞响应地址: {', '.join(map(str, ambiguous))}")
    if not found:
        print("没有得到有效响应。检查电机动力电源、USB-RS485 的 A/B、波特率和共地。")
    else:
        print(f"扫描完成：{len(found)} 个可确认地址。")
    return found


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="USB-RS485 串口，例如 COM55")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"波特率，默认 {DEFAULT_BAUD}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="电脑直连 USB-RS485 控制 Y42 电机；默认只做扫描/查询，运动命令需确认。"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("ports", help="列出可用串口")

    scan_parser = commands.add_parser("scan", help="扫描未知电机 ID（只读状态，不让电机运动）")
    add_common(scan_parser)
    scan_parser.add_argument("--start", type=int, default=MIN_ID)
    scan_parser.add_argument("--end", type=int, default=MAX_ID)
    scan_parser.add_argument("--verbose", action="store_true", help="打印每个超时/异常响应")

    status_parser = commands.add_parser("status", help="读取一台电机遥测")
    add_common(status_parser)
    status_parser.add_argument("--id", type=int, required=True)

    id_parser = commands.add_parser("set-id", help="把已知旧 ID 改为新 ID 并保存")
    add_common(id_parser)
    id_parser.add_argument("--old", type=int, required=True, help="当前电机 ID")
    id_parser.add_argument("--new", type=int, required=True, help="新的唯一 ID")
    id_parser.add_argument("--yes", action="store_true", help="跳过确认")

    for name, help_text in (("enable", "使能并锁轴"), ("disable", "失能并松轴"), ("stop", "发送急停")):
        motor_parser = commands.add_parser(name, help=help_text)
        add_common(motor_parser)
        motor_parser.add_argument("--id", type=int, required=True)
        if name == "enable":
            motor_parser.add_argument("--yes", action="store_true")

    position_parser = commands.add_parser("position", help="单电机位置模式测试（Y42 X 固件）")
    add_common(position_parser)
    position_parser.add_argument("--id", type=int, required=True)
    position_parser.add_argument("--degrees", type=float, required=True, help="位置角度，单位度；含义由 --mode 决定")
    position_parser.add_argument("--direction", type=int, choices=(0, 1), default=0)
    position_parser.add_argument("--rpm", type=float, default=5.0)
    position_parser.add_argument("--acceleration", type=int, default=100)
    position_parser.add_argument("--mode", type=int, choices=(0, 1, 2), default=2)
    position_parser.add_argument("--yes", action="store_true")

    torque_parser = commands.add_parser("torque", help="单电机力矩测试（Y42 X 固件）")
    add_common(torque_parser)
    torque_parser.add_argument("--id", type=int, required=True)
    torque_parser.add_argument("--direction", type=int, choices=(0, 1), default=0)
    torque_parser.add_argument("--current", type=int, default=100, help="电流 mA，默认 100")
    torque_parser.add_argument("--slope", type=int, default=2000)
    torque_parser.add_argument("--max-rpm", type=float, default=10.0)
    torque_parser.add_argument("--yes", action="store_true")

    speed_parser = commands.add_parser("speed", help="单电机速度测试（Y42 X 固件）")
    add_common(speed_parser)
    speed_parser.add_argument("--id", type=int, required=True)
    speed_parser.add_argument("--rpm", type=float, required=True)
    speed_parser.add_argument("--direction", type=int, choices=(0, 1), default=0)
    speed_parser.add_argument("--acceleration", type=int, default=100)
    speed_parser.add_argument("--yes", action="store_true")

    raw_parser = commands.add_parser("raw", help="发送原始十六进制帧")
    add_common(raw_parser)
    raw_parser.add_argument("hex_frame", help="例如 '01 3A 6B'")
    raw_parser.add_argument("--yes", action="store_true")

    return parser


def run(args: argparse.Namespace) -> int:
    if args.command == "ports":
        return print_serial_ports()

    port = choose_port(args.port)
    if args.command == "scan":
        if not 0 <= args.start <= args.end <= MAX_ID:
            raise ValueError("扫描范围必须满足 0 <= start <= end <= 255")
    elif hasattr(args, "id"):
        check_id(args.id)

    with open_serial(port, args.baud) as connection:
        if args.command == "scan":
            if args.start == 0:
                print("ID 0 是广播地址，不发送广播扫描帧；实际扫描 ID 1..%d" % args.end)
            scan(connection, max(MIN_ID, args.start), args.end, args.verbose)
            return 0

        if args.command == "status":
            print_telemetry(read_telemetry(connection, args.id))
            return 0

        if args.command == "set-id":
            check_id(args.old, "旧 ID")
            check_id(args.new, "新 ID")
            confirm(
                f"即将把 ID {args.old} 改为 {args.new}。请确认总线上只有这一台使用旧 ID 的电机。",
                args.yes,
            )
            reply = set_motor_id(connection, args.old, args.new)
            print(f"TX: {args.old:02X} AE 4B 01 {args.new:02X} 6B")
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            if command_accepted(reply, args.old, 0xAE) or (reply and reply[1] == 0xAE):
                print("设置 ID 命令已收到响应。断电重启电机后，再扫描确认新 ID。")
            else:
                print("未得到标准确认；请勿继续改名，先检查 A/B、供电和当前旧 ID。")
            return 0

        if args.command in ("enable", "position", "torque", "speed"):
            confirm("该命令可能使电机动作，请确认机构无人员接触并已准备硬件急停。", args.yes)

        if args.command == "enable":
            reply = transact(connection, frame(args.id, 0xF3, 0xAB, 0x01, 0x00, FRAME_END))
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "disable":
            reply = transact(connection, frame(args.id, 0xFE, 0x98, 0x00, FRAME_END))
            print(f"stop RX: {hex_bytes(reply) if reply else '(timeout)'}")
            reply = transact(connection, frame(args.id, 0xF3, 0xAB, 0x00, 0x00, FRAME_END))
            print(f"disable RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "stop":
            reply = transact(connection, frame(args.id, 0xFE, 0x98, 0x00, FRAME_END))
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "position":
            tx = position_frame(args.id, args.direction, args.rpm, args.acceleration, args.degrees, args.mode)
            reply = transact(connection, tx)
            print(f"TX: {hex_bytes(tx)}")
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "torque":
            tx = torque_frame(args.id, args.direction, args.current, args.slope, args.max_rpm)
            reply = transact(connection, tx)
            print(f"TX: {hex_bytes(tx)}")
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "speed":
            tx = speed_frame(args.id, args.direction, args.rpm, args.acceleration)
            reply = transact(connection, tx)
            print(f"TX: {hex_bytes(tx)}")
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

        if args.command == "raw":
            confirm("即将发送原始 RS485 帧，请确认帧内容和机构安全。", args.yes)
            tx = parse_hex_frame(args.hex_frame)
            reply = transact(connection, tx, first_byte_timeout=0.08)
            print(f"TX: {hex_bytes(tx)}")
            print(f"RX: {hex_bytes(reply) if reply else '(timeout)'}")
            return 0

    raise RuntimeError(f"未知命令: {args.command}")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except (ValueError, RuntimeError, OSError, serial.SerialException) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n已取消。")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
