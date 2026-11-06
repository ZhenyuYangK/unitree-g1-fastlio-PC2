#!/usr/bin/env bash
set -Eeuo pipefail
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export G1_WEB_MODE="${G1_WEB_MODE:-ros_control}"
case "${G1_WEB_MODE}" in
  local|mock) export G1_WEB_HOST="${G1_WEB_HOST:-127.0.0.1}" ;;
  *) export G1_WEB_HOST="${G1_WEB_HOST:-0.0.0.0}" ;;
esac
export G1_WEB_PORT="${G1_WEB_PORT:-8080}"

if [[ -z "${G1_WEB_TOKEN:-}" ]] && {
  [[ "${G1_WEB_MODE}" == ros_readonly || "${G1_WEB_MODE}" == ros_control ]] ||
  [[ "${G1_WEB_HOST}" != 127.0.0.1 && "${G1_WEB_HOST}" != localhost && "${G1_WEB_HOST}" != ::1 ]];
}; then
  if [[ ! -t 0 ]]; then
    printf '[ERROR] 非交互启动请预先设置 G1_WEB_TOKEN。\n' >&2
    exit 2
  fi
  while [[ -z "${G1_WEB_TOKEN:-}" ]]; do
    if ! IFS= read -r -s -p '请输入网页访问口令 web_token（输入隐藏）: ' G1_WEB_TOKEN; then
      printf '\n[ERROR] 未读取到口令，取消启动。\n' >&2
      exit 2
    fi
    printf '\n'
    [[ -n "${G1_WEB_TOKEN}" ]] || printf '[提示] 口令不能为空，请重新输入。\n' >&2
  done
fi
export G1_WEB_TOKEN="${G1_WEB_TOKEN:-}"

source "${CONSOLE_DIR}/../script/g1_env.sh"
if [[ "${G1_WEB_MODE}" == ros_readonly || "${G1_WEB_MODE}" == ros_control ]]; then
  g1_source_environment
fi
export PYTHONPATH="${CONSOLE_DIR}/backend${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -x "${CONSOLE_DIR}/.venv/bin/python" ]]; then
  WEB_PYTHON="${G1_WEB_PYTHON:-${CONSOLE_DIR}/.venv/bin/python}"
else
  WEB_PYTHON="${G1_WEB_PYTHON:-/usr/bin/python3}"
fi
"${WEB_PYTHON}" -c 'import sys; assert sys.version_info >= (3, 8), "PC2 console requires Python 3.8+"'
if [[ "${G1_WEB_MODE}" == ros_readonly || "${G1_WEB_MODE}" == ros_control ]]; then
  "${WEB_PYTHON}" -c 'import rclpy' || { echo '[ERROR] 请用 /usr/bin/python3 -m venv --system-site-packages 创建网页虚拟环境'; exit 2; }
fi

# Validate configuration and listening address before advertising URLs.
"${WEB_PYTHON}" - <<'PY'
import socket
import sys
from g1_web.config import Settings

settings = Settings.from_env()
try:
    addresses = socket.getaddrinfo(settings.host, settings.port, type=socket.SOCK_STREAM,
                                   flags=socket.AI_PASSIVE)
    for family, socktype, protocol, _, address in addresses:
        with socket.socket(family, socktype, protocol) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(address)
except OSError as exc:
    sys.exit('[ERROR] 无法监听 {}:{}（{}）。若已有网页运行，请先在原终端 Ctrl+C 停止，或设置 G1_WEB_PORT。'
             .format(settings.host, settings.port, exc))
PY

printf '\n[启动] 模式=%s  监听=%s:%s\n' "${G1_WEB_MODE}" "${G1_WEB_HOST}" "${G1_WEB_PORT}"
WEB_ACCESS_HOST="${G1_WEB_HOST}"
case "${G1_WEB_HOST}" in
  0.0.0.0|::) WEB_ACCESS_HOST=127.0.0.1 ;;
esac
[[ "${WEB_ACCESS_HOST}" != *:* ]] || WEB_ACCESS_HOST="[${WEB_ACCESS_HOST}]"
WEB_URL="http://${WEB_ACCESS_HOST}:${G1_WEB_PORT}"
printf 'PC2 浏览器访问：%s\n' "${WEB_URL}"
if [[ "${G1_WEB_HOST}" == 0.0.0.0 || "${G1_WEB_HOST}" == :: ]]; then
  WEB_ADDRESS_COUNT=0
  while read -r WEB_IFACE WEB_ADDRESS; do
    case "${WEB_IFACE}" in lo|docker*|veth*|br-*|virbr*) continue ;; esac
    printf '其他设备访问（%s）：http://%s:%s\n' "${WEB_IFACE}" "${WEB_ADDRESS%/*}" "${G1_WEB_PORT}"
    WEB_ADDRESS_COUNT=$((WEB_ADDRESS_COUNT + 1))
  done < <(ip -o -4 addr show up scope global 2>/dev/null | awk '{print $2, $4}' || true)
  (( WEB_ADDRESS_COUNT > 0 )) || printf '[提示] 未发现可用局域网 IPv4 地址，请检查网络连接。\n'
elif [[ "${G1_WEB_HOST}" == 127.0.0.1 || "${G1_WEB_HOST}" == localhost || "${G1_WEB_HOST}" == ::1 ]]; then
  printf '[提示] 当前仅限 PC2 本机访问；其他设备访问请设置 G1_WEB_HOST=0.0.0.0。\n'
else
  printf '其他设备访问：%s\n' "${WEB_URL}"
fi
printf '其他设备需与对应网卡网络互通，并使用刚才的口令登录。Ctrl+C 停止网页服务。\n\n'

exec "${WEB_PYTHON}" -m g1_web
