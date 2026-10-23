#!/usr/bin/env bash
# 只构建，不启动节点；保留原 build/install/log。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '用法：bash script/g1_build.sh [colcon build 的附加参数]'; exit 0
fi
g1_load_ros
[[ "$(uname -m)" == aarch64 ]] || { g1_error 'PC2 构建入口要求在 aarch64 目标机运行'; exit 1; }
python3 -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 8) else "PC2 构建要求 Python 3.8")'
command -v colcon >/dev/null || { g1_error '缺少 colcon'; exit 1; }
# 不允许附加参数绕过平台产物隔离。
for arg in "$@"; do
  case "$arg" in
    --build-base*|--install-base*|--log-base*|--base-paths*|--merge-install)
      g1_error '构建目录通过 G1_BUILD_DIR/G1_INSTALL_DIR/G1_LOG_DIR 配置'; exit 2 ;;
  esac
done
python3 "${SCRIPT_DIR}/g1_platform.py" check --offline >/dev/null
cd "$G1_WS"
mkdir -p "$G1_BUILD_DIR" "$G1_INSTALL_DIR" "$G1_LOG_DIR"
python3 "${SCRIPT_DIR}/g1_build_guard.py"
# Livox SDK 是纯 CMake 项目；显式本地安装，不写 /usr/local、不链接旧架构库。
cmake -S "$G1_WS/src/Livox-SDK2" -B "$G1_BUILD_DIR/livox_sdk2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$G1_INSTALL_DIR/livox_sdk2"
cmake --build "$G1_BUILD_DIR/livox_sdk2" --parallel "${G1_BUILD_JOBS:-2}"
cmake --install "$G1_BUILD_DIR/livox_sdk2"
export CMAKE_PREFIX_PATH="$G1_INSTALL_DIR/livox_sdk2${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="$G1_INSTALL_DIR/livox_sdk2/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
# 只扫描 src；不会递归编译 backup、旧 build 和 third_party 示例。
exec colcon --log-base "$G1_LOG_DIR" build \
  --base-paths "$G1_WS/src" --packages-ignore livox_sdk2 \
  --build-base "$G1_BUILD_DIR" --install-base "$G1_INSTALL_DIR" \
  --executor sequential "$@" \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=foxy -DCMAKE_BUILD_TYPE=Release
