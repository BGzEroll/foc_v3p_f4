#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PYTHON_SCRIPT="${SCRIPT_DIR}/uart_terminal.py"
readonly DEFAULT_PORT="${UART_PORT:-/dev/serial/by-id/usb-CMSIS-DAP_STM32_CMSIS-DAP_CMSIS-DAP-if01}"
readonly DEFAULT_BAUD="${UART_BAUD:-460800}"
readonly USBIP_REMOTE="${UART_USBIP_REMOTE:-192.168.114.137}"
readonly USBIP_BUS_ID="${UART_USBIP_BUS_ID:-2-2}"
readonly USBIP_TOOL="/usr/sbin/usbip"

has_custom_port=false
for argument in "$@"; do
    case "${argument}" in
        --port|--port=*)
            has_custom_port=true
            break
            ;;
    esac
done

if [[ "${has_custom_port}" == false && ! -e "${DEFAULT_PORT}" ]]; then
    if ! /usr/bin/lsusb -d 0416:5051 >/dev/null 2>&1; then
        if [[ ! -x "${USBIP_TOOL}" ]]; then
            echo "错误：DAPLink 未连接，且未找到 ${USBIP_TOOL}" >&2
            exit 1
        fi

        echo "未检测到 DAPLink，正在通过 USBIP 连接 ${USBIP_REMOTE}/${USBIP_BUS_ID}" >&2
        sudo "${USBIP_TOOL}" attach \
            -r "${USBIP_REMOTE}" \
            -b "${USBIP_BUS_ID}"
    fi

    for _ in {1..40}; do
        if [[ -e "${DEFAULT_PORT}" ]]; then
            break
        fi
        sleep 0.25
    done
fi

if [[ "${has_custom_port}" == false && ! -e "${DEFAULT_PORT}" ]]; then
    echo "错误：未找到串口 ${DEFAULT_PORT}" >&2
    echo "可以使用 --port /dev/ttyXXX 指定其他串口" >&2
    exit 1
fi

exec /usr/bin/python3 "${PYTHON_SCRIPT}" \
    --port "${DEFAULT_PORT}" \
    --baud "${DEFAULT_BAUD}" \
    "$@"
