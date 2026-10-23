#!/usr/bin/env bash
# 导航点入口：相邻 Python 辅助脚本复用当前工作空间的源码。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
HELPER="${SCRIPT_DIR}/g1_waypoints_tool.py"
[[ -f "$HELPER" ]] || { echo "[ERROR] 缺少 $HELPER" >&2; exit 2; }

# 仅采样命令需要 ROS；离线命令无需启动机器人或加载 ROS。
needs_ros=0
args=("$@")
index=0
while (( index < ${#args[@]} )); do
  case "${args[index]}" in
    --file|--missions-file) index=$((index + 2)) ;;
    --file=*|--missions-file=*) index=$((index + 1)) ;;
    record|add|update) needs_ros=1; break ;;
    *) break ;;
  esac
done
for arg in "$@"; do
  case "$arg" in -h|--help) needs_ros=0 ;; esac
done
if [[ "$needs_ros" == 1 ]]; then
  g1_source_environment
fi
exec /usr/bin/python3 "$HELPER" "$@"
