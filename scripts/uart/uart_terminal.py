#!/usr/bin/env python3

"""无需第三方依赖的 Linux 交互式串口终端。"""

import argparse
import contextlib
import fcntl
import os
import select
import selectors
import sys
import termios
import tty
from collections.abc import Iterator


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-CMSIS-DAP_STM32_CMSIS-DAP_CMSIS-DAP-if01"
)
DEFAULT_BAUD_RATE = 460800
EXIT_CHARACTER = 0x1D
READ_BUFFER_SIZE = 4096


def available_baud_rates() -> dict[int, int]:
    """返回当前 Python termios 支持的波特率及其常量。"""
    baud_rates: dict[int, int] = {}

    for baud_rate in (
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        460800,
        500000,
        576000,
        921600,
        1000000,
        1152000,
        1500000,
        2000000,
    ):
        termios_value = getattr(termios, f"B{baud_rate}", None)
        if termios_value is not None:
            baud_rates[baud_rate] = termios_value

    return baud_rates


BAUD_RATES = available_baud_rates()


def parse_arguments() -> argparse.Namespace:
    """解析串口终端命令行参数。"""
    parser = argparse.ArgumentParser(
        description=(
            "通过 Linux TTY 与 STM32 UART 交互；键盘输入会立即发送，"
            "串口数据会实时显示。"
        )
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"串口设备路径，默认：{DEFAULT_PORT}",
    )
    parser.add_argument(
        "--baud",
        type=int,
        choices=sorted(BAUD_RATES),
        default=DEFAULT_BAUD_RATE,
        help=f"波特率，默认：{DEFAULT_BAUD_RATE}",
    )
    parser.add_argument(
        "--newline",
        choices=("crlf", "lf", "cr"),
        default="crlf",
        help="按 Enter 时发送的换行符，默认：crlf",
    )
    parser.add_argument(
        "--local-echo",
        action="store_true",
        help="在本地显示键盘输入；对端自身不回显时使用",
    )
    parser.add_argument(
        "--hex",
        action="store_true",
        help="把接收数据按十六进制显示，而不是直接显示文本",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="只监视串口，不读取或发送键盘输入；按 Ctrl+C 退出",
    )
    return parser.parse_args()


def newline_bytes(newline_mode: str) -> bytes:
    """根据换行模式生成串口换行字节。"""
    return {
        "crlf": b"\r\n",
        "lf": b"\n",
        "cr": b"\r",
    }[newline_mode]


def configure_serial(file_descriptor: int, baud_rate: int) -> None:
    """把串口配置为指定波特率的 8-N-1 Raw 模式。"""
    attributes = termios.tcgetattr(file_descriptor)
    attributes[0] = 0
    attributes[1] = 0
    attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attributes[3] = 0
    attributes[4] = BAUD_RATES[baud_rate]
    attributes[5] = BAUD_RATES[baud_rate]
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0

    termios.tcsetattr(file_descriptor, termios.TCSANOW, attributes)
    termios.tcflush(file_descriptor, termios.TCIOFLUSH)


def lock_serial(file_descriptor: int, port: str) -> None:
    """独占串口，避免多个终端同时读写同一设备。"""
    try:
        fcntl.flock(file_descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exception:
        raise RuntimeError(f"串口已被其他进程占用：{port}") from exception


def write_all(file_descriptor: int, data: bytes) -> None:
    """把全部字节写入非阻塞文件描述符。"""
    view = memoryview(data)

    while view:
        try:
            written_size = os.write(file_descriptor, view)
            view = view[written_size:]
        except BlockingIOError:
            _, writable, _ = select.select(
                [],
                [file_descriptor],
                [],
                1.0,
            )
            if not writable:
                raise TimeoutError("等待串口或终端可写超时")


def render_received(data: bytes, hexadecimal: bool) -> None:
    """以文本原始字节或十六进制形式显示接收内容。"""
    if hexadecimal:
        hexadecimal_text = " ".join(f"{value:02X}" for value in data)
        sys.stdout.write(f"[RX {len(data):4d}] {hexadecimal_text}\n")
        sys.stdout.flush()
        return

    write_all(sys.stdout.fileno(), data)


def render_local_input(data: bytes) -> None:
    """在启用本地回显时显示键盘输入。"""
    for value in data:
        if value in (0x0A, 0x0D):
            write_all(sys.stdout.fileno(), b"\r\n")
        elif value in (0x08, 0x7F):
            write_all(sys.stdout.fileno(), b"\b \b")
        elif value >= 0x20:
            write_all(sys.stdout.fileno(), bytes((value,)))


def transform_keyboard_input(data: bytes, newline_mode: str) -> bytes:
    """处理退出快捷键和 Enter 换行转换。"""
    transformed = bytearray()
    line_ending = newline_bytes(newline_mode)

    for value in data:
        if value == EXIT_CHARACTER:
            break
        if value in (0x0A, 0x0D):
            transformed.extend(line_ending)
        else:
            transformed.append(value)

    return bytes(transformed)


@contextlib.contextmanager
def keyboard_cbreak_mode(file_descriptor: int) -> Iterator[None]:
    """临时关闭终端行缓冲，并在退出时恢复原始设置。"""
    original_attributes = termios.tcgetattr(file_descriptor)
    tty.setcbreak(file_descriptor, termios.TCSANOW)

    try:
        yield
    finally:
        termios.tcsetattr(
            file_descriptor,
            termios.TCSANOW,
            original_attributes,
        )


def print_session_header(arguments: argparse.Namespace) -> None:
    """向标准错误输出当前会话配置和快捷键。"""
    input_mode = "只读监视" if arguments.monitor else "交互收发"
    echo_mode = "开启" if arguments.local_echo else "关闭"

    print(
        "\n"
        "UART 终端已连接\n"
        f"  设备：{arguments.port}\n"
        f"  配置：{arguments.baud}-8-N-1\n"
        f"  模式：{input_mode}\n"
        f"  Enter：{arguments.newline.upper()}\n"
        f"  本地回显：{echo_mode}\n"
        "  退出：Ctrl+]，也可按 Ctrl+C\n",
        file=sys.stderr,
        flush=True,
    )


def run_terminal(arguments: argparse.Namespace) -> None:
    """运行串口接收显示和键盘发送事件循环。"""
    if not arguments.monitor and not sys.stdin.isatty():
        raise RuntimeError("交互模式要求标准输入连接到终端")

    serial_file_descriptor = os.open(
        arguments.port,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    received_total = 0
    transmitted_total = 0

    try:
        lock_serial(serial_file_descriptor, arguments.port)
        configure_serial(serial_file_descriptor, arguments.baud)

        selector = selectors.DefaultSelector()
        selector.register(serial_file_descriptor, selectors.EVENT_READ, "serial")

        input_context: contextlib.AbstractContextManager[None]
        if arguments.monitor:
            input_context = contextlib.nullcontext()
        else:
            selector.register(sys.stdin.fileno(), selectors.EVENT_READ, "stdin")
            input_context = keyboard_cbreak_mode(sys.stdin.fileno())

        print_session_header(arguments)

        with input_context:
            running = True

            while running:
                for key, _ in selector.select():
                    if key.data == "serial":
                        data = os.read(serial_file_descriptor, READ_BUFFER_SIZE)
                        if not data:
                            raise ConnectionError("串口设备已断开")

                        render_received(data, arguments.hex)
                        received_total += len(data)
                        continue

                    keyboard_data = os.read(
                        sys.stdin.fileno(),
                        READ_BUFFER_SIZE,
                    )
                    if not keyboard_data:
                        running = False
                        break

                    exit_requested = EXIT_CHARACTER in keyboard_data
                    serial_data = transform_keyboard_input(
                        keyboard_data,
                        arguments.newline,
                    )

                    if serial_data:
                        write_all(serial_file_descriptor, serial_data)
                        transmitted_total += len(serial_data)
                        if arguments.local_echo:
                            visible_input = keyboard_data.split(
                                bytes((EXIT_CHARACTER,)),
                                1,
                            )[0]
                            render_local_input(visible_input)

                    if exit_requested:
                        running = False
                        break

        selector.close()
    finally:
        os.close(serial_file_descriptor)
        print(
            "\n"
            f"UART 终端已退出：接收 {received_total} 字节，"
            f"发送 {transmitted_total} 字节",
            file=sys.stderr,
            flush=True,
        )


def main() -> int:
    """执行串口终端并把错误转换为清晰的退出状态。"""
    arguments = parse_arguments()

    try:
        run_terminal(arguments)
    except KeyboardInterrupt:
        return 0
    except (ConnectionError, OSError, RuntimeError) as exception:
        print(f"错误：{exception}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
