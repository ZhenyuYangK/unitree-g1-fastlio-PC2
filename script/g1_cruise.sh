#!/usr/bin/env bash
# 巡航兼容入口；状态机由 g1_task_manager.cruise 维护。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1_env.sh"
export PYTHONPATH="$G1_WS/src/g1_task_manager${PYTHONPATH:+:$PYTHONPATH}"
needs_ros=0
if [[ "${1:-}" == start ]]; then
  for arg in "$@"; do [[ "$arg" != --execute ]] || needs_ros=1; done
  for arg in "$@"; do
    case "$arg" in --help|-h) needs_ros=0 ;; esac
  done
fi
if [[ "$needs_ros" == 1 ]]; then
  g1_source_environment
fi
exec /usr/bin/python3 -m g1_task_manager.cruise "$@"
