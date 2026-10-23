#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
WS="$G1_WS"
RUNTIME_DIR="$WS/runtime/mapping"
LOG_DIR="$RUNTIME_DIR/logs"
PID_FILE="$RUNTIME_DIR/mapping.pids"
SESSION_FILE="$RUNTIME_DIR/session.json"
RUNTIME_CFG="$RUNTIME_DIR/mid360_mapping.yaml"
SOURCE_CFG="$WS/src/FAST_LIO_ROS2/config/mid360.yaml"
RVIZ_CFG="$WS/src/FAST_LIO_ROS2/rviz/fastlio.rviz"

die() { g1_error "$*"; exit 1; }
processes() { python3 "$SCRIPT_DIR/g1_process.py" "$@"; }
start_process() {
  local name="$1"; shift
  processes start --file "$PID_FILE" --name "$name" --log "$LOG_DIR/$name.log" -- "$@"
}

assert_not_running() {
  local rc=0
  processes status --file "$1" || rc=$?
  [[ "$rc" == 1 ]] || die '已有运行进程或进程归属待核实，请先停止对应系统'
}

wait_for() {
  local kind="$1" name="$2" deadline=$((SECONDS + $3))
  while (( SECONDS < deadline )); do
    if timeout 4 ros2 "$kind" list 2>/dev/null | grep -Fxq "$name"; then return 0; fi
    sleep 1
  done
  die "未检测到 $name，请查看 $LOG_DIR"
}

start_all() {
  g1_lock_stack
  assert_not_running "$WS/runtime/navigation/navigation.pids"
  assert_not_running "$WS/runtime/navigation/real_bridge.pid"
  assert_not_running "$PID_FILE"
  g1_source_environment
  for package in livox_ros_driver2 fast_lio; do
    ros2 pkg prefix "$package" >/dev/null || die "ROS 包不存在：$package"
  done
  local nodes
  nodes="$(timeout 8 ros2 node list)" || die '无法检查现有 ROS 节点'
  if grep -Eq '/(livox_lidar_publisher|laserMapping|fastlio_mapping|controller_server|planner_server|g1_relocalization|map_server)$' <<<"$nodes"; then
    die '已有建图/定位/导航节点，需由原启动方停止后再启动'
  fi
  g1_prepare_livox_config
  mkdir -p "$LOG_DIR"
  processes stop --file "$PID_FILE"
  python3 "$SCRIPT_DIR/g1_mapping_state.py" create --source "$SOURCE_CFG" \
    --config "$RUNTIME_CFG" --session "$SESSION_FILE"
  start_process livox ros2 launch livox_ros_driver2 msg_MID360_launch.py
  wait_for topic /livox/lidar 35
  wait_for topic /livox/imu 20
  start_process fast_lio ros2 launch fast_lio mapping.launch.py \
    config_path:="$RUNTIME_DIR" config_file:="$(basename "$RUNTIME_CFG")" rviz:=false
  wait_for topic /Odometry 40
  wait_for service /map_save 30
  g1_start_rviz "$RVIZ_CFG"
  echo '[OK] 建图系统已启动；save 保存，finish 保存成功后停止'
}

save_map() {
  g1_source_environment
  local rc=0
  processes status --file "$PID_FILE" --name fast_lio || rc=$?
  [[ "$rc" == 0 ]] || die '没有可验证的本工作空间建图进程'
  python3 "$SCRIPT_DIR/g1_mapping_save.py" --session "$SESSION_FILE" --receipt "$RUNTIME_DIR/save_receipt.json"
}

stop_all() {
  processes stop --file "$PID_FILE"
  echo '[OK] 建图进程已停止'
}

case "${1:-start}" in
  start) start_all ;;
  save) g1_lock_stack; save_map ;;
  finish) g1_lock_stack; save_map; stop_all ;;
  stop) g1_lock_stack; stop_all ;;
  restart) g1_lock_stack; stop_all; start_all ;;
  status) processes status --file "$PID_FILE" ;;
  map-path) python3 "$SCRIPT_DIR/g1_mapping_state.py" path --session "$SESSION_FILE" ;;
  logs) for file in "$LOG_DIR"/*.log; do [[ ! -f "$file" ]] || tail -n 60 "$file"; done ;;
  help|-h|--help) echo "用法：$0 start|save|finish|stop|restart|status|logs|map-path" ;;
  *) g1_error '未知命令'; exit 2 ;;
esac
