#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
WS="$G1_WS"
RUNTIME="$WS/runtime/navigation"
CFG="$RUNTIME/bridge_real.yaml"
NAV_CONFIG="$RUNTIME/navigation_config.json"
NAV_PROFILE="${G1_NAV_PROFILE:-safe}"
LOG_DIR="$RUNTIME/logs"
PID_FILE="$RUNTIME/real_bridge.pid"
LOG_FILE="$LOG_DIR/real_bridge.log"
AUDIO_PID_FILE="$RUNTIME/audio_bridge.pid"
AUDIO_LOG_FILE="$LOG_DIR/audio_bridge.log"
AUDIO_CREATED=0
mkdir -p "$LOG_DIR"

load_env() { g1_source_environment; }
processes() { python3 "$SCRIPT_DIR/g1_process.py" "$@"; }

check_nav() {
    local rc=0
    processes status --file "$WS/runtime/mapping/mapping.pids" || rc=$?
    [[ "$rc" == 1 ]] || { g1_error '建图进程运行中或归属待核实'; return 1; }
    processes status --file "$RUNTIME/navigation.pids" --name nav2
    processes status --file "$RUNTIME/navigation.pids" --name relocalization
    python3 "$SCRIPT_DIR/g1_wait_localization.py" --once --timeout 5 --tf-frame nav_base
    check_bridge_mode nav
}

check_bridge_mode() {
    python3 "$SCRIPT_DIR/g1_navigation_readiness.py" --manifest "$NAV_CONFIG" \
        --profile "$NAV_PROFILE" --mode "$1" "${@:2}"
}

stop_audio_bridge() {
    # g1_process verifies PID, kernel start time, command and workspace ownership.
    # External instances have no record here and remain with their launcher.
    processes stop --file "$AUDIO_PID_FILE"
}

start_audio_bridge() {
    local rc=0 volume="${G1_AUDIO_VOLUME:--1}"
    AUDIO_CREATED=0
    [[ "$volume" =~ ^(-1|0|[1-9][0-9]?|100)$ ]] || {
        g1_error 'G1_AUDIO_VOLUME 必须为 -1（保留当前音量）或 0..100'
        return 1
    }
    # Refuse duplicate or incompatible servers; only absence permits spawning.
    python3 -m g1_task_manager.audio_readiness --timeout 2 --probe || rc=$?
    if [[ "$rc" == 0 ]]; then
        echo '[OK] 复用已核验的真实语音桥'
        return
    fi
    [[ "$rc" == 3 ]] || return "$rc"
    rc=0
    processes status --file "$AUDIO_PID_FILE" || rc=$?
    if [[ "$rc" == 0 ]]; then
        g1_error '已有本工作空间语音进程，但服务不可见；请检查 Domain 或先执行 stop'
        return 1
    fi
    [[ "$rc" == 1 ]] || return "$rc"
    processes start --file "$AUDIO_PID_FILE" --name audio_bridge --log "$AUDIO_LOG_FILE" -- \
        ros2 run g1_audio_bridge g1_audio_bridge --ros-args -p dry_run:=false -p "volume:=$volume"
    AUDIO_CREATED=1
    if ! python3 -m g1_task_manager.audio_readiness --timeout 10; then
        stop_audio_bridge
        tail -n 80 "$AUDIO_LOG_FILE"
        return 1
    fi
    echo '[OK] 真实语音桥已启动；默认保留机器人当前音量'
}

start_bridge() {
    g1_lock_stack
    load_env
    check_nav
    local rc=0
    processes status --file "$PID_FILE" || rc=$?
    if [[ "$rc" == 0 ]]; then
        check_bridge_mode real
        start_audio_bridge
        echo '[OK] 复用已核验的本工作空间真机桥'
        return
    fi
    [[ "$rc" == 1 ]] || return "$rc"
    local nodes
    nodes="$(timeout 8 ros2 node list)" || { g1_error '无法检查已有桥节点'; return 1; }
    if grep -Fxq /g1_cmd_vel_dry_run <<<"$nodes"; then
        g1_error '已有同名桥节点，请由原启动方核实模式并停止'
        return 1
    fi
    start_audio_bridge
    echo '[WARN] 真机桥启动：运动模式须由操作者切换；安全门保持关闭'
    if ! processes start --file "$PID_FILE" --name real_bridge --log "$LOG_FILE" -- \
        ros2 run g1_loco_bridge g1_cmd_vel_dry_run --ros-args --params-file "$CFG"; then
        if [[ "$AUDIO_CREATED" == 1 ]]; then stop_audio_bridge; fi
        return 1
    fi
    if ! check_bridge_mode real; then
        processes stop --file "$PID_FILE"
        if [[ "$AUDIO_CREATED" == 1 ]]; then stop_audio_bridge; fi
        tail -n 80 "$LOG_FILE"
        return 1
    fi
    echo '[OK] 真机桥后端、定位联锁和速度参数已核验；安全门保持关闭'
}

enable_bridge() {
    g1_lock_stack
    processes status --file "$PID_FILE" --name real_bridge
    load_env
    check_nav
    check_bridge_mode real --enable
}

disable_bridge() {
    load_env
    timeout 8 ros2 service call /g1_loco_bridge/enable_real_nav \
        std_srvs/srv/SetBool "{data: false}"
}

stop_bridge() {
    g1_lock_stack
    local rc=0
    processes status --file "$PID_FILE" || rc=$?
    if [[ "$rc" == 0 ]]; then
        if load_env; then disable_bridge || true; fi
    elif [[ "$rc" != 1 ]]; then
        return "$rc"
    fi
    processes stop --file "$PID_FILE"
    stop_audio_bridge
    echo '[OK] 真机安全桥已停止'
}

status() {
    load_env
    processes status --file "$PID_FILE"
    check_nav
    check_bridge_mode real
    python3 -m g1_task_manager.audio_readiness --timeout 3
    if [[ -f "$AUDIO_PID_FILE" ]]; then
        echo '[INFO] 语音桥由本工作空间管理；stop 将停止该进程'
    else
        echo '[INFO] 外部语音桥由原启动方停止'
    fi
}

main() {
    case "${1:-}" in
        start) start_bridge ;;
        enable) enable_bridge ;;
        disable) g1_lock_stack; processes status --file "$PID_FILE" --name real_bridge; disable_bridge ;;
        stop) stop_bridge ;;
        status) status ;;
        logs) tail -n 150 "$LOG_FILE" "$AUDIO_LOG_FILE" ;;
        -h|--help|help) echo "用法：$0 start|enable|disable|status|logs|stop" ;;
        *) echo "用法：$0 start|enable|disable|status|logs|stop"; return 2 ;;
    esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
