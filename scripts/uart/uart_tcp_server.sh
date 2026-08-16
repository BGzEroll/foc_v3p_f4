#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PYTHON_SCRIPT="${SCRIPT_DIR}/uart_tcp_server.py"
readonly DEFAULT_PORT="${UART_PORT:-/dev/serial/by-id/usb-CMSIS-DAP_STM32_CMSIS-DAP_CMSIS-DAP-if01}"
readonly DEFAULT_BAUD="${UART_BAUD:-460800}"
readonly USBIP_REMOTE="${UART_USBIP_REMOTE:-192.168.114.137}"
readonly USBIP_BUS_ID="${UART_USBIP_BUS_ID:-}"
readonly USBIP_TOOL="/usr/sbin/usbip"

find_usbip_bus_id()
{
    if [[ -n "${USBIP_BUS_ID}" ]]; then
        printf '%s\n' "${USBIP_BUS_ID}"
        return 0
    fi

    local remote_devices
    if ! remote_devices="$("${USBIP_TOOL}" list \
        -r "${USBIP_REMOTE}" 2>&1)"; then
        echo "错误：无法查询 USBIP 服务器 ${USBIP_REMOTE}" >&2
        echo "${remote_devices}" >&2
        return 1
    fi

    local bus_id
    bus_id="$(printf '%s\n' "${remote_devices}" | awk '
        /^[[:space:]]*[0-9]+-[0-9]+([.][0-9]+)*:/ {
            current_bus_id = $1
            sub(/:$/, "", current_bus_id)
        }
        tolower($0) ~ /cmsis-dap|0416:5051/ {
            if(current_bus_id != "")
            {
                print current_bus_id
                exit
            }
        }
    ')"

    if [[ -z "${bus_id}" ]]; then
        echo "错误：${USBIP_REMOTE} 未导出 CMSIS-DAP（0416:5051）" >&2
        echo "${remote_devices}" >&2
        return 1
    fi

    printf '%s\n' "${bus_id}"
}

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

        selected_bus_id="$(find_usbip_bus_id)"
        echo "未检测到 DAPLink，正在通过 USBIP 连接 ${USBIP_REMOTE}/${selected_bus_id}" >&2
        sudo "${USBIP_TOOL}" attach \
            -r "${USBIP_REMOTE}" \
            -b "${selected_bus_id}"
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

exec /usr/bin/python3 -B "${PYTHON_SCRIPT}" \
    --port "${DEFAULT_PORT}" \
    --baud "${DEFAULT_BAUD}" \
    "$@"
