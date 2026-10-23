#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
WS="$G1_WS"

CFG="$WS/src/g1_loco_bridge/config/g1_loco_bridge_nav2_real.yaml"

RUNTIME="$WS/runtime/navigation"
LOG_DIR="$RUNTIME/logs"
PID_FILE="$RUNTIME/real_bridge.pid"
LOG_FILE="$LOG_DIR/real_bridge.log"

mkdir -p "$LOG_DIR"

load_env() { g1_source_environment; }
processes() { python3 "$SCRIPT_DIR/g1_process.py" "$@"; }

check_nav()
{
    for n in \
        controller_server \
        planner_server \
        recoveries_server \
        bt_navigator
    do
        state="$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null || true)"

        if ! grep -Eqi '^active[[:space:]]*\[3\]' <<<"$state"; then
            echo "[ERROR] $n 不是 active"
            exit 1
        fi
    done

    ros2 topic list | grep -Fxq /cmd_vel_nav || {
        echo "[ERROR] /cmd_vel_nav 不存在"
        exit 1
    }

    python3 "$SCRIPT_DIR/g1_wait_localization.py" --once --timeout 5 --tf-frame nav_base
    echo "[OK] Nav2 和实时定位已准备好"
}

start_bridge()
{
    g1_lock_stack
    load_env
    check_nav

    local rc=0
    processes status --file "$PID_FILE" || rc=$?
    if [[ "$rc" == 0 ]]; then
        local existing_guard
        existing_guard="$(timeout 8 ros2 param get /g1_cmd_vel_dry_run require_localization)"
        grep -q True <<<"$existing_guard" || {
            echo '[ERROR] 已运行的桥缺少定位联锁，请重建并重启'
            return 1
        }
        echo '[OK] 当前工作空间真机桥已运行'
        return
    fi
    [[ "$rc" == 1 ]] || return "$rc"
    if timeout 8 ros2 node list | grep -Fxq /g1_cmd_vel_dry_run; then
        g1_error '已有同名桥节点，请由原启动方核实模式并停止'
        return 1
    fi

    echo
    echo "=========================================="
    echo "G1 REAL NAVIGATION"
    echo "=========================================="
    echo
    echo "请确认："
    echo "  1. 机器人运动模式已经由你手动切换"
    echo "  2. 周围没有人员和障碍物"
    echo "  3. 遥控器/急停在手边"
    echo
    echo "安全门启动后仍然保持关闭。"
    echo "=========================================="
    echo

    processes start --file "$PID_FILE" --name real_bridge --log "$LOG_FILE" -- \
        ros2 run g1_loco_bridge g1_cmd_vel_dry_run --ros-args --params-file "$CFG"

    sleep 3

    ros2 node list |
        grep -Fxq /g1_cmd_vel_dry_run || {
            echo "[ERROR] real bridge 启动失败"
            tail -n 80 "$LOG_FILE"
            exit 1
        }

    value="$(ros2 param get \
        /g1_cmd_vel_dry_run \
        dry_run)"

    grep -q False <<<"$value" || {
        echo "[ERROR] dry_run 不是 False"
        stop_bridge
        exit 1
    }

    topic="$(ros2 param get \
        /g1_cmd_vel_dry_run \
        cmd_vel_topic)"

    grep -q /cmd_vel_nav <<<"$topic" || {
        echo "[ERROR] 输入不是 /cmd_vel_nav"
        stop_bridge
        exit 1
    }

    local guard
    guard="$(timeout 8 ros2 param get /g1_cmd_vel_dry_run require_localization)"
    grep -q True <<<"$guard" || {
        echo '[ERROR] 真机桥未启用定位联锁，请重建并重启'
        stop_bridge
        exit 1
    }

    echo "[OK] 真机安全桥已启动"
    echo "[OK] 安全门当前仍关闭"
}

enable_bridge()
{
    g1_lock_stack
    processes status --file "$PID_FILE"
    load_env
    check_nav
    local guard
    guard="$(timeout 8 ros2 param get /g1_cmd_vel_dry_run require_localization)"
    grep -q True <<<"$guard" || { echo '[ERROR] 真机桥未启用定位联锁'; return 1; }

    echo
    echo "[WARN] 即将允许 Nav2 控制真实 G1"
    echo "[WARN] 机器人模式必须已经由你手动切换"
    echo

    timeout 8 ros2 service call \
        /g1_loco_bridge/enable_real_nav \
        std_srvs/srv/SetBool \
        "{data: true}"
}

disable_bridge()
{
    load_env

    timeout 8 ros2 service call \
        /g1_loco_bridge/enable_real_nav \
        std_srvs/srv/SetBool \
        "{data: false}" \
        2>/dev/null || true
}

stop_bridge()
{
    g1_lock_stack
    local rc=0
    processes status --file "$PID_FILE" || rc=$?
    if [[ "$rc" == 0 ]]; then
        if load_env; then disable_bridge || true; fi
    elif [[ "$rc" != 1 ]]; then
        return "$rc"
    fi
    processes stop --file "$PID_FILE"
    echo "[OK] 真机安全桥已停止"
}

status()
{
    load_env

    echo "===== Nav2 ====="

    for n in \
        controller_server \
        planner_server \
        recoveries_server \
        bt_navigator
    do
        printf "%-22s " "$n"
        ros2 lifecycle get "/$n" 2>/dev/null ||
            echo "未运行"
    done

    echo
    echo "===== Real bridge ====="

    if ros2 node list 2>/dev/null |
        grep -Fxq /g1_cmd_vel_dry_run
    then
        ros2 param get \
            /g1_cmd_vel_dry_run \
            dry_run

        ros2 param get \
            /g1_cmd_vel_dry_run \
            cmd_vel_topic

        echo
        ros2 service list |
            grep g1_loco_bridge || true
    else
        echo "未运行"
    fi
}

logs()
{
    tail -n 150 "$LOG_FILE"
}

case "${1:-}" in

    start)
        start_bridge
        ;;

    enable)
        enable_bridge
        ;;

    disable)
        g1_lock_stack
        processes status --file "$PID_FILE"
        disable_bridge
        ;;

    stop)
        stop_bridge
        ;;

    status)
        status
        ;;

    logs)
        logs
        ;;

    -h|--help|help)
        echo "用法：$0 start|enable|disable|status|logs|stop"
        ;;

    *)
        echo "用法："
        echo "  $0 start"
        echo "  $0 enable"
        echo "  $0 disable"
        echo "  $0 status"
        echo "  $0 logs"
        echo "  $0 stop"
        exit 1
        ;;
esac
