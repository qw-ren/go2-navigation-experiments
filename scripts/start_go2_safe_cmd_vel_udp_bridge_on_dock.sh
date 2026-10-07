#!/usr/bin/env bash
set -euo pipefail

# Run on the Go2 dock. ROS 2 side only: /go2_safe/cmd_vel -> local UDP.
# This does not import Unitree SDK and does not directly command Go2 motors.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
: "${CYCLONEDDS_URI:?Set CYCLONEDDS_URI to a private local CycloneDDS XML file URI}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY

set +u
source "${ROS_SETUP}"
set -u

exec /usr/bin/python3 "${SCRIPT_DIR}/go2_safe_cmd_vel_udp_bridge.py" --ros-args \
  -p input_topic:="${INPUT_CMD_TOPIC:-/go2_safe/cmd_vel}" \
  -p status_topic:="${UDP_BRIDGE_STATUS_TOPIC:-/go2_safe/udp_bridge_status}" \
  -p udp_host:="${GO2_UDP_HOST:-127.0.0.1}" \
  -p udp_port:="${GO2_UDP_PORT:-15001}" \
  -p max_linear_x:="${GO2_UDP_MAX_LINEAR_X:-0.10}" \
  -p max_angular_z:="${GO2_UDP_MAX_ANGULAR_Z:-0.30}"
