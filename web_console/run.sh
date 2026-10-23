#!/usr/bin/env bash
set -Eeuo pipefail
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export G1_WEB_MODE="${G1_WEB_MODE:-local}"
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
exec "${WEB_PYTHON}" -m g1_web
