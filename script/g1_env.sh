#!/usr/bin/env bash
# PC2 公共配置。source 只读取配置、定义函数，不加载 ROS 或启动节点。
G1_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

g1_error() { printf '[ERROR] %s\n' "$*" >&2; }

g1_read_config() {
  local key config="${G1_LOCAL_ENV:-${G1_SCRIPT_DIR}/local.env}"
  local -A overrides=()
  # 调用者导出的配置优先于本机文件，包括显式设置为空的可选 underlay。
  while IFS= read -r key; do
    case "$key" in
      G1_*|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_LOCALHOST_ONLY)
        overrides["$key"]="${!key}" ;;
    esac
  done < <(compgen -e)
  if [[ -f "$config" ]]; then
    # shellcheck disable=SC1090
    source "$config" || return 1
  elif [[ -n "${G1_LOCAL_ENV:-}" ]]; then
    g1_error "找不到 G1_LOCAL_ENV：$config"; return 1
  fi
  for key in "${!overrides[@]}"; do
    export "$key=${overrides[$key]}"
  done
  export G1_WS="${G1_WS:-$(cd -- "${G1_SCRIPT_DIR}/.." && pwd)}"
  G1_WS="$(cd -- "$G1_WS" && pwd)" || return 1
  export G1_ROS_DISTRO="${G1_ROS_DISTRO:-foxy}"
  export G1_ROS_SETUP="${G1_ROS_SETUP:-/opt/ros/${G1_ROS_DISTRO}/setup.bash}"
  export G1_INSTALL_DIR="${G1_INSTALL_DIR:-${G1_WS}/install_pc2}"
  export G1_BUILD_DIR="${G1_BUILD_DIR:-${G1_WS}/build_pc2}"
  export G1_LOG_DIR="${G1_LOG_DIR:-${G1_WS}/log_pc2}"
  export G1_PCL_SETUP="${G1_PCL_SETUP:-}"
  export G1_UNITREE_ROS2_SETUP="${G1_UNITREE_ROS2_SETUP:-}"
  export G1_RMW_SETUP="${G1_RMW_SETUP:-}"
  export G1_DDS_LIB_DIR="${G1_DDS_LIB_DIR:-}"
  export G1_RMW_RELEASE_TEST="${G1_RMW_RELEASE_TEST:-0}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export G1_ENABLE_RVIZ="${G1_ENABLE_RVIZ:-1}"
  export G1_LOCALIZATION_TIMEOUT_SEC="${G1_LOCALIZATION_TIMEOUT_SEC:-180.0}"
  export G1_NETWORK_INTERFACE="${G1_NETWORK_INTERFACE:-}"
  export G1_HOST_IP="${G1_HOST_IP:-}"
  export G1_LIDAR_IP="${G1_LIDAR_IP:-}"
  for key in G1_WAYPOINTS_FILE G1_MISSIONS_FILE G1_MAPPING_OUTPUT G1_BUILD_JOBS CYCLONEDDS_URI ROS_LOCALHOST_ONLY; do
    [[ ! -v "$key" ]] || export "$key"
  done
  export G1_MAP_PCD="${G1_MAP_PCD:-${G1_WS}/maps/map.pcd}"
  export G1_MAP_PGM="${G1_MAP_PGM:-${G1_WS}/maps/map_multilayer.pgm}"
  export G1_MAP_YAML="${G1_MAP_YAML:-${G1_WS}/maps/map_multilayer.yaml}"
  [[ "$G1_ROS_DISTRO" == foxy ]] || {
    g1_error 'PC2 平台入口只支持 Foxy；不能用 Humble 构建覆盖目标产物'; return 1;
  }
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] && (( 10#$ROS_DOMAIN_ID <= 232 )) || {
    g1_error 'ROS_DOMAIN_ID 必须在 0–232 范围内'; return 1;
  }
  for key in G1_INSTALL_DIR G1_BUILD_DIR G1_LOG_DIR G1_ROS_SETUP G1_PCL_SETUP G1_UNITREE_ROS2_SETUP G1_RMW_SETUP G1_DDS_LIB_DIR G1_MAP_PCD G1_MAP_PGM G1_MAP_YAML G1_WAYPOINTS_FILE G1_MISSIONS_FILE; do
    [[ -n "${!key:-}" ]] || continue
    [[ "${!key}" == /* ]] || export "$key=${G1_WS}/${!key}"
  done
}

g1_reject_mixed_ros() {
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != foxy ]]; then
    g1_error "当前已加载 ${ROS_DISTRO}，请使用未加载其他 ROS 版本的新终端"; return 1
  fi
  local key entry
  for key in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do
    local -a entries=()
    IFS=: read -ra entries <<<"${!key:-}"
    for entry in "${entries[@]}"; do
      if [[ "$entry" =~ /opt/ros/([^/]+) && "${BASH_REMATCH[1]}" != foxy ]]; then
        g1_error "$key 混入其他 ROS 版本：$entry"; return 1
      fi
    done
  done
}

g1_source_file() {
  local path="$1" had_nounset=0 result=0
  [[ -r "$path" ]] || { g1_error "找不到环境入口：$path"; return 1; }
  [[ "$-" != *u* ]] || had_nounset=1
  set +u
  # shellcheck disable=SC1090
  source "$path" || result=$?
  if (( had_nounset )); then set -u; fi
  (( result == 0 )) || return "$result"
  g1_reject_mixed_ros
}

g1_load_ros() {
  g1_reject_mixed_ros || return 1
  g1_source_file "$G1_ROS_SETUP" || return 1
  [[ "${ROS_DISTRO:-}" == foxy ]] || { g1_error '环境入口未加载 Foxy'; return 1; }
  local path
  for path in "$G1_PCL_SETUP" "$G1_UNITREE_ROS2_SETUP"; do
    [[ -z "$path" ]] || g1_source_file "$path" || return 1
  done
  if [[ "$G1_RMW_RELEASE_TEST" == 1 ]]; then
    [[ -n "$G1_RMW_SETUP" && -n "$G1_DDS_LIB_DIR" ]] || {
      g1_error 'release 调试必须成对配置 G1_RMW_SETUP 和 G1_DDS_LIB_DIR'; return 1;
    }
  fi
  if [[ -n "$G1_RMW_SETUP" || -n "$G1_DDS_LIB_DIR" ]]; then
    [[ -n "$G1_RMW_SETUP" && -n "$G1_DDS_LIB_DIR" && "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]] || {
      g1_error '自定义 CycloneDDS/RMW 必须成对指定且使用 rmw_cyclonedds_cpp'; return 1;
    }
    g1_source_file "$G1_RMW_SETUP" || return 1
    [[ -r "$G1_DDS_LIB_DIR/libddsc.so" || -r "$G1_DDS_LIB_DIR/libddsc.so.0" ]] || {
      g1_error "缺少 CycloneDDS 共享库：$G1_DDS_LIB_DIR"; return 1;
    }
    export LD_LIBRARY_PATH="${G1_DDS_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  # Foxy/旧 CycloneDDS 使用 NetworkInterfaceAddress，避免照搬 Humble XML。
  if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && -z "${CYCLONEDDS_URI:-}" && -n "${G1_NETWORK_INTERFACE:-}" ]]; then
    [[ "$G1_NETWORK_INTERFACE" =~ ^[a-zA-Z0-9_.:-]+$ ]] || { g1_error '无效网卡名称'; return 1; }
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${G1_NETWORK_INTERFACE}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
  fi
  export ROS_LOG_DIR="${G1_WS}/runtime/ros_logs"
  python3 "${G1_SCRIPT_DIR}/g1_platform.py" rmw || return 1
}

g1_source_environment() {
  g1_load_ros || return 1
  # local_setup 不加载旧 build/install 里冻结的原机器 underlay 链。
  g1_source_file "${G1_INSTALL_DIR}/local_setup.bash" || return 1
  if [[ -d "${G1_INSTALL_DIR}/livox_sdk2/lib" ]]; then
    export LD_LIBRARY_PATH="${G1_INSTALL_DIR}/livox_sdk2/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  python3 "${G1_SCRIPT_DIR}/g1_platform.py" rmw
}

g1_prepare_livox_config() {
  export G1_LIVOX_CONFIG="${G1_WS}/runtime/livox/MID360_config.json"
  python3 "${G1_SCRIPT_DIR}/g1_livox_config.py"
}

g1_lock_stack() {
  local lock="${G1_WS}/runtime/stack.lock"
  if [[ "$(readlink "/proc/$$/fd/9" 2>/dev/null || true)" == "$lock" ]]; then
    flock -n 9 || { g1_error '无法取得管理锁'; return 1; }
    return 0
  fi
  mkdir -p "${G1_WS}/runtime"
  exec 9>"$lock"
  flock -n 9 || { g1_error '另一个建图/导航管理命令正在执行'; return 1; }
}

g1_start_rviz() {
  [[ "$G1_ENABLE_RVIZ" == 1 ]] || return 0
  [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { g1_error '没有图形显示；可设置 G1_ENABLE_RVIZ=0'; return 1; }
  start_process rviz rviz2 -d "$1"
}

g1_read_config
