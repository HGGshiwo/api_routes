#!/usr/bin/env bash
# api_routes yaw 上报测试入口: source ROS 环境后运行 pytest。
# 用法: ./run_test.sh [pytest 额外参数, 如 -k test_loc_base_yaw_report]
set -uo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$PKG_DIR/pytest/logs"
mkdir -p "$LOG_DIR"

source /opt/ros/noetic/setup.bash
source "$PKG_DIR/../../devel/setup.bash"

cleanup() {
  # 只清理本脚本拉起的进程; 已在运行的外部节点原样保留
  pkill -f "api_routes_node _web_port" 2>/dev/null
  pkill -f "roslaunch.*api_routes" 2>/dev/null
}
trap cleanup EXIT

cd "$PKG_DIR"
python3 -m pytest pytest/ -v "$@"
RC=$?
echo "pytest exit code: $RC"
exit $RC
