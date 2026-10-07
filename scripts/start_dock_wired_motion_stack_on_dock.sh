#!/usr/bin/env bash
set -euo pipefail

# Run on the Go2 dock.
#
# Dock motion stack for wired or Wi-Fi ROS discovery:
# - MID-360 driver
# - dock Fast-LIO
# - body -> base_link static TF
# - camera_init -> base_footprint projector
# - safety gate
# - UDP bridge
# - clean Unitree SDK worker
#
# Nav2 is started separately by start_dock_nav2_on_dock.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/go2_dock_stack_$(date +%Y%m%d_%H%M%S)}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
GO2_WS_SETUP="${GO2_WS_SETUP:-${HOME}/go2_ws/install/setup.bash}"
FASTLIO_SETUP="${FASTLIO_SETUP:-${HOME}/ws_fastlio/install/setup.bash}"
GO2_LINK_MODE="${GO2_LINK_MODE:-wired}"
: "${CYCLONEDDS_URI:?Set CYCLONEDDS_URI to a private local CycloneDDS XML file URI}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY

for setup_file in "${ROS_SETUP}" "${GO2_WS_SETUP}" "${FASTLIO_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing setup file: ${setup_file}" >&2
    exit 2
  fi
done

mkdir -p "${LOG_DIR}"

set +u
source "${ROS_SETUP}"
source "${GO2_WS_SETUP}"
source "${FASTLIO_SETUP}"
set -u

children=()
tail_pid=""
cleanup_started=0

start_bg() {
  local name="$1"
  shift
  echo "Starting ${name}; log=${LOG_DIR}/${name}.log"
  ("$@") >"${LOG_DIR}/${name}.log" 2>&1 &
  children+=("$!")
}

cleanup() {
  if [[ "${cleanup_started}" == "1" ]]; then
    return 0
  fi
  cleanup_started=1
  trap - EXIT INT TERM

  echo
  echo "Stopping dock wired motion stack ..."

  if [[ -n "${tail_pid}" ]] && kill -0 "${tail_pid}" 2>/dev/null; then
    kill "${tail_pid}" 2>/dev/null || true
  fi

  local pid
  for pid in "${children[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done

  sleep 3

  for pid in "${children[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
  echo "Dock stack stopped. Logs kept in ${LOG_DIR}"
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    if ros2 topic list --no-daemon 2>/dev/null | grep -qx "${topic}"; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for ${topic}" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local timeout_s="${2:-20}"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    if ros2 service list --no-daemon 2>/dev/null | grep -qx "${service}"; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for ${service}" >&2
  return 1
}

echo "=== Go2 dock wired motion stack ==="
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "LOG_DIR=${LOG_DIR}"
echo "UNITREE_IFACE=${UNITREE_IFACE:-eth0}"
echo "GO2_REAL_SEND=${GO2_REAL_SEND:-0}"
echo "MAX_LINEAR_X=${MAX_LINEAR_X:-0.10}"
echo "MAX_ANGULAR_Z=${MAX_ANGULAR_Z:-0.30}"
echo "GO2_UDP_MAX_LINEAR_X=${GO2_UDP_MAX_LINEAR_X:-0.10}"
echo "GO2_UDP_MAX_ANGULAR_Z=${GO2_UDP_MAX_ANGULAR_Z:-0.30}"
echo "GO2_UNITREE_MAX_LINEAR_X=${GO2_UNITREE_MAX_LINEAR_X:-0.10}"
echo "GO2_UNITREE_MAX_ANGULAR_Z=${GO2_UNITREE_MAX_ANGULAR_Z:-0.30}"

ros2 daemon stop 2>/dev/null || true

start_bg mid360 ros2 launch livox_ros_driver2 msg_MID360_launch.py
wait_for_topic /livox/lidar 40
wait_for_topic /livox/imu 20

start_bg fastlio env GO2_LINK_MODE="${GO2_LINK_MODE}" "${SCRIPT_DIR}/start_dock_fastlio_on_dock.sh"
wait_for_topic /Odometry 40

start_bg body_static "${SCRIPT_DIR}/start_go2_body_static_tf.sh"
start_bg footprint_projector /usr/bin/python3 "${SCRIPT_DIR}/project_fastlio_odom_to_base_footprint.py"

start_bg safe_gate /usr/bin/python3 "${SCRIPT_DIR}/safe_cmd_vel_gate.py" --ros-args \
  -p input_topic:="${INPUT_CMD_TOPIC:-/nav2_debug/cmd_vel}" \
  -p output_topic:="${OUTPUT_CMD_TOPIC:-/go2_safe/cmd_vel}" \
  -p enabled_on_start:=false \
  -p max_linear_x:="${MAX_LINEAR_X:-0.10}" \
  -p max_angular_z:="${MAX_ANGULAR_Z:-0.30}" \
  -p max_accel_linear_x:="${MAX_ACCEL_LINEAR_X:-0.20}" \
  -p max_accel_angular_z:="${MAX_ACCEL_ANGULAR_Z:-0.60}" \
  -p input_timeout_s:="${INPUT_TIMEOUT_S:-0.35}" \
  -p publish_rate_hz:="${PUBLISH_RATE_HZ:-20.0}"
wait_for_service /go2_safe/set_enabled 20

start_bg udp_bridge env \
  GO2_LINK_MODE="${GO2_LINK_MODE}" \
  GO2_UDP_MAX_LINEAR_X="${GO2_UDP_MAX_LINEAR_X:-0.10}" \
  GO2_UDP_MAX_ANGULAR_Z="${GO2_UDP_MAX_ANGULAR_Z:-0.30}" \
  "${SCRIPT_DIR}/start_go2_safe_cmd_vel_udp_bridge_on_dock.sh"

start_bg unitree_worker env \
  GO2_REAL_SEND="${GO2_REAL_SEND:-0}" \
  GO2_ALLOW_REAL_SEND="${GO2_ALLOW_REAL_SEND:-}" \
  UNITREE_IFACE="${UNITREE_IFACE:-eth0}" \
  GO2_UNITREE_MAX_LINEAR_X="${GO2_UNITREE_MAX_LINEAR_X:-0.10}" \
  GO2_UNITREE_MAX_ANGULAR_Z="${GO2_UNITREE_MAX_ANGULAR_Z:-0.30}" \
  CALL_SWITCH_JOYSTICK_OFF_ON_START="${CALL_SWITCH_JOYSTICK_OFF_ON_START:-true}" \
  RESTORE_JOYSTICK_ON_SHUTDOWN="${RESTORE_JOYSTICK_ON_SHUTDOWN:-true}" \
  CALL_CLASSIC_WALK_ON_START="${CALL_CLASSIC_WALK_ON_START:-true}" \
  IGNORE_CLASSIC_WALK_START_FAILURE="${IGNORE_CLASSIC_WALK_START_FAILURE:-true}" \
  CLASSIC_WALK_OFF_ON_SHUTDOWN="${CLASSIC_WALK_OFF_ON_SHUTDOWN:-true}" \
  CALL_BALANCE_STAND_ON_START="${CALL_BALANCE_STAND_ON_START:-false}" \
  "${SCRIPT_DIR}/start_go2_unitree_sport_udp_worker_on_dock.sh"

if [[ "${ENABLE_SAFE_GATE_ON_START:-false}" == "true" || "${ENABLE_SAFE_GATE_ON_START:-false}" == "1" ]]; then
  echo "Enabling safety gate ..."
  ros2 service call /go2_safe/reset_estop std_srvs/srv/Trigger "{}" >/dev/null || true
  ros2 service call /go2_safe/set_enabled std_srvs/srv/SetBool "{data: true}" >/dev/null
else
  echo "Safety gate is running but disabled. Enable with:"
  echo "  ros2 service call /go2_safe/set_enabled std_srvs/srv/SetBool \"{data: true}\""
fi

echo
echo "Dock stack started. Keep this terminal open."
echo "Logs: ${LOG_DIR}"
echo "Tail follows. Ctrl-C stops the stack."
echo

tail -n +1 -F "${LOG_DIR}"/*.log &
tail_pid="$!"
wait
