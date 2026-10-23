#!/usr/bin/env bash
# 导航点入口：相邻 Python 辅助脚本复用当前工作空间的源码。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export G1_WS="${G1_WS:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
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
  if [[ -f "$G1_WS/script/g1_run_verified.sh" ]]; then
    exec bash "$G1_WS/script/g1_run_verified.sh" /usr/bin/python3 "$HELPER" "$@"
  fi
  # 精简源码包未包含 verified 脚本，使用现有 Foxy 工作空间环境。
  if [[ "${ROS_DISTRO:-}" != foxy ]]; then
    [[ -f /opt/ros/foxy/setup.bash ]] || { echo '[ERROR] 未找到 ROS2 Foxy' >&2; exit 2; }
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
  fi
  [[ -f "$G1_WS/install/setup.bash" ]] || { echo '[ERROR] 缺少 install/setup.bash，请在已构建的 PC2 工作空间运行' >&2; exit 2; }
  set +u
  source "$G1_WS/install/setup.bash"
  set -u
fi
exec /usr/bin/python3 "$HELPER" "$@"
