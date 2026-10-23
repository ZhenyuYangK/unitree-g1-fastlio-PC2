#!/usr/bin/env bash
# 只读自检；不会 ros2 run/launch、调用运动服务或创建雷达配置。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
case "${1:-}" in
  --offline)
    exec python3 "${SCRIPT_DIR}/g1_platform.py" check --offline ;;
  -h|--help)
    echo '用法：bash script/g1_check.sh [--offline]（只读，不启动节点）'; exit 0 ;;
  '') ;;
  *) g1_error '未知参数'; exit 2 ;;
esac
g1_load_ros
if [[ -f "$G1_INSTALL_DIR/local_setup.bash" ]]; then
  g1_source_file "$G1_INSTALL_DIR/local_setup.bash"
fi
python3 "${SCRIPT_DIR}/g1_platform.py" check
