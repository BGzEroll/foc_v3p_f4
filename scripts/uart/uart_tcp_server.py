#!/usr/bin/env python3

"""在串口和单个 TCP 客户端之间进行双向原始数据透传。"""

import argparse
import errno
import os
import selectors
import socket
import sys

from uart_terminal import (
    BAUD_RATES,
    DEFAULT_BAUD_RATE,
    DEFAULT_PORT,
    READ_BUFFER_SIZE,
    configure_serial,
    lock_serial,
    write_all,
)


DEFAULT_LISTEN_ADDRESS = "0.0.0.0"
DEFAULT_LISTEN_PORT = 1919
MAX_PENDING_BYTES = 1024 * 1024


def parse_arguments() -> argparse.Namespace:
    """解析串口 TCP 服务命令行参数。"""
    parser = argparse.ArgumentParser(
        description="把 STM32 串口与 TCP 客户端进行双向原始数据透传。",
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
        help=f"串口波特率，默认：{DEFAULT_BAUD_RATE}",
    )
    parser.add_argument(
        "--listen-address",
        default=DEFAULT_LISTEN_ADDRESS,
        help=f"TCP 监听地址，默认：{DEFAULT_LISTEN_ADDRESS}",
    )
    parser.add_argument(
        "--listen-port",
        type=int,
        choices=range(1, 65536),
        default=DEFAULT_LISTEN_PORT,
        metavar="PORT",
        help=f"TCP 监听端口，默认：{DEFAULT_LISTEN_PORT}",
    )
    return parser.parse_args()


def configure_listener(address: str, port: int) -> socket.socket:
    """创建 TCP 监听套接字。"""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((address, port))
    listener.listen(1)
    listener.setblocking(False)
    return listener


def close_client(
    selector: selectors.BaseSelector,
    client: socket.socket,
) -> None:
    """注销并关闭客户端连接。"""
    try:
        selector.unregister(client)
    except (KeyError, ValueError):
        pass

    try:
        client.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass

    client.close()


def client_description(client: socket.socket) -> str:
    """生成人类可读的客户端地址。"""
    host, port = client.getpeername()[:2]
    return f"{host}:{port}"


def set_client_events(
    selector: selectors.BaseSelector,
    client: socket.socket,
    pending_data: bytearray,
) -> None:
    """根据发送缓冲状态更新客户端事件。"""
    events = selectors.EVENT_READ
    if pending_data:
        events |= selectors.EVENT_WRITE
    selector.modify(client, events, "client")


def run_server(arguments: argparse.Namespace) -> None:
    """运行串口与 TCP 双向透传服务。"""
    serial_file_descriptor = os.open(
        arguments.port,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )
    listener: socket.socket | None = None
    client: socket.socket | None = None
    selector = selectors.DefaultSelector()
    pending_tcp_data = bytearray()
    serial_to_tcp_total = 0
    tcp_to_serial_total = 0

    try:
        lock_serial(serial_file_descriptor, arguments.port)
        configure_serial(serial_file_descriptor, arguments.baud)

        listener = configure_listener(
            arguments.listen_address,
            arguments.listen_port,
        )
        selector.register(serial_file_descriptor, selectors.EVENT_READ, "serial")
        selector.register(listener, selectors.EVENT_READ, "listener")

        print(
            "\n"
            "UART TCP 服务已启动\n"
            f"  串口：{arguments.port}\n"
            f"  配置：{arguments.baud}-8-N-1\n"
            f"  监听：{arguments.listen_address}:{arguments.listen_port}\n"
            "  模式：原始字节双向透传，仅允许一个客户端\n"
            "  退出：Ctrl+C\n",
            flush=True,
        )

        while True:
            for key, event_mask in selector.select():
                if key.data == "listener":
                    new_client, _ = listener.accept()
                    new_client.setsockopt(
                        socket.IPPROTO_TCP,
                        socket.TCP_NODELAY,
                        1,
                    )
                    new_client.setblocking(False)

                    if client is not None:
                        print(
                            "拒绝额外客户端："
                            f"{client_description(new_client)}",
                            flush=True,
                        )
                        new_client.close()
                        continue

                    client = new_client
                    pending_tcp_data.clear()
                    selector.register(client, selectors.EVENT_READ, "client")
                    print(
                        f"SSCOM 已连接：{client_description(client)}",
                        flush=True,
                    )
                    continue

                if key.data == "serial":
                    try:
                        serial_data = os.read(
                            serial_file_descriptor,
                            READ_BUFFER_SIZE,
                        )
                    except BlockingIOError:
                        continue

                    if not serial_data:
                        raise ConnectionError("串口设备已断开")

                    if client is None:
                        continue

                    pending_tcp_data.extend(serial_data)
                    serial_to_tcp_total += len(serial_data)

                    if len(pending_tcp_data) > MAX_PENDING_BYTES:
                        peer = client_description(client)
                        close_client(selector, client)
                        client = None
                        pending_tcp_data.clear()
                        print(
                            f"SSCOM {peer} 接收过慢，连接已关闭",
                            flush=True,
                        )
                        continue

                    set_client_events(selector, client, pending_tcp_data)
                    continue

                if client is None or key.fileobj is not client:
                    continue

                client_disconnected = False

                if event_mask & selectors.EVENT_READ:
                    try:
                        tcp_data = client.recv(READ_BUFFER_SIZE)
                    except BlockingIOError:
                        tcp_data = None
                    except ConnectionError:
                        tcp_data = b""

                    if tcp_data == b"":
                        client_disconnected = True
                    elif tcp_data:
                        write_all(serial_file_descriptor, tcp_data)
                        tcp_to_serial_total += len(tcp_data)

                if (
                    not client_disconnected
                    and event_mask & selectors.EVENT_WRITE
                    and pending_tcp_data
                ):
                    try:
                        sent_size = client.send(pending_tcp_data)
                        del pending_tcp_data[:sent_size]
                    except BlockingIOError:
                        pass
                    except ConnectionError:
                        client_disconnected = True

                if client_disconnected:
                    peer = client_description(client)
                    close_client(selector, client)
                    client = None
                    pending_tcp_data.clear()
                    print(f"SSCOM 已断开：{peer}", flush=True)
                    continue

                set_client_events(selector, client, pending_tcp_data)
    finally:
        if client is not None:
            close_client(selector, client)
        if listener is not None:
            try:
                selector.unregister(listener)
            except (KeyError, ValueError):
                pass
            listener.close()
        try:
            selector.unregister(serial_file_descriptor)
        except (KeyError, ValueError):
            pass
        selector.close()
        os.close(serial_file_descriptor)

        print(
            "\n"
            "UART TCP 服务已退出："
            f"串口→TCP {serial_to_tcp_total} 字节，"
            f"TCP→串口 {tcp_to_serial_total} 字节",
            flush=True,
        )


def main() -> int:
    """执行串口 TCP 服务并输出清晰错误。"""
    arguments = parse_arguments()

    try:
        run_server(arguments)
    except KeyboardInterrupt:
        return 0
    except (ConnectionError, OSError, RuntimeError) as exception:
        if isinstance(exception, OSError) and exception.errno == errno.EADDRINUSE:
            print(
                f"错误：TCP 端口 {arguments.listen_port} 已被占用",
                file=sys.stderr,
            )
        else:
            print(f"错误：{exception}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
