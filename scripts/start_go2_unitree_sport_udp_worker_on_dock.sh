#!/usr/bin/env bash
set -euo pipefail

# Run on the Go2 dock. Clean Unitree SDK2 Python worker.
# This process intentionally does not source ROS and does not import rclpy.
#
# Real sending requires:
#   GO2_REAL_SEND=1
#   GO2_ALLOW_REAL_SEND=I_UNDERSTAND_GO2_WILL_MOVE

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY
unset ROS_DOMAIN_ID
unset RMW_IMPLEMENTATION

if [[ -n "${UNITREE_SDK_PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${UNITREE_SDK_PYTHONPATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi
export UNITREE_IFACE="${UNITREE_IFACE:-eth0}"

exec /usr/bin/python3 "${SCRIPT_DIR}/go2_unitree_sport_udp_worker.py"
