#!/usr/bin/env bash
set -euo pipefail

# Run on the Go2 dock, after start_dock_wired_motion_stack_on_dock.sh is up.
#
# Starts on the dock:
# - static map -> camera_init same-start transform
# - map_server
# - planner_server, controller_server, bt_navigator
#
# It intentionally keeps cmd_vel remapped to /nav2_debug/cmd_vel, which is then
# filtered by the already-running dock safety gate.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/go2_dock_nav2_$(date +%Y%m%d_%H%M%S)}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
GO2_WS_SETUP="${GO2_WS_SETUP:-${HOME}/go2_ws/install/setup.bash}"
FASTLIO_SETUP="${FASTLIO_SETUP:-${HOME}/ws_fastlio/install/setup.bash}"
GO2_LINK_MODE="${GO2_LINK_MODE:-wired}"
: "${MAP_YAML:?Set MAP_YAML to a private local Nav2 map YAML file}"
NAV2_PARAMS="${NAV2_PARAMS:-${REPO_ROOT}/nav2/nav2_params_real_fastlio_foxy.yaml}"
BT_XML="${BT_XML:-${REPO_ROOT}/nav2/real_nav_to_pose_no_recovery.xml}"
STOP_EXISTING_NAV2="${STOP_EXISTING_NAV2:-false}"
ENABLE_DOCK_SAFE_GATE="${ENABLE_DOCK_SAFE_GATE:-false}"
START_MAP_TO_CAMERA_STATIC="${START_MAP_TO_CAMERA_STATIC:-true}"

MAP_TO_CAMERA_X="${MAP_TO_CAMERA_X:-0.0}"
MAP_TO_CAMERA_Y="${MAP_TO_CAMERA_Y:-0.0}"
MAP_TO_CAMERA_Z="${MAP_TO_CAMERA_Z:-0.0}"
MAP_TO_CAMERA_QX="${MAP_TO_CAMERA_QX:-0.0}"
MAP_TO_CAMERA_QY="${MAP_TO_CAMERA_QY:-0.0}"
MAP_TO_CAMERA_QZ="${MAP_TO_CAMERA_QZ:-0.0}"
MAP_TO_CAMERA_QW="${MAP_TO_CAMERA_QW:-1.0}"

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

if [[ ! -f "${MAP_YAML}" ]]; then
  echo "Missing map yaml: ${MAP_YAML}" >&2
  exit 2
fi

if [[ ! -f "${NAV2_PARAMS}" ]]; then
  echo "Missing Nav2 params: ${NAV2_PARAMS}" >&2
  exit 2
fi
if [[ ! -f "${BT_XML}" ]]; then
  echo "Missing Nav2 behavior tree: ${BT_XML}" >&2
  exit 2
fi

mkdir -p "${LOG_DIR}"

set +u
source "${ROS_SETUP}"
source "${GO2_WS_SETUP}"
source "${FASTLIO_SETUP}"
set -u

children=()
tail_pid=""
cleanup_started=0
startup_complete=0

print_recent_logs() {
  local log
  for log in "${LOG_DIR}"/*.log; do
    [[ -f "${log}" ]] || continue
    echo
    echo "===== tail -80 ${log} =====" >&2
    tail -80 "${log}" >&2 || true
  done
}

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
  echo "Stopping dock Nav2 stack ..."

  if ros2 service list --no-daemon 2>/dev/null | grep -qx "/go2_safe/stop"; then
    ros2 service call /go2_safe/stop std_srvs/srv/Trigger "{}" >/dev/null || true
  fi

  if [[ -n "${tail_pid}" ]] && kill -0 "${tail_pid}" 2>/dev/null; then
    kill "${tail_pid}" 2>/dev/null || true
  fi

  local pid
  for pid in "${children[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  sleep 2

  for pid in "${children[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
  echo "Dock Nav2 stack stopped. Logs kept in ${LOG_DIR}"
}
trap cleanup EXIT INT TERM

on_error() {
  local line_no="$1"
  echo "Dock Nav2 stack failed near line ${line_no}." >&2
  if [[ "${startup_complete}" != "1" ]]; then
    print_recent_logs
  fi
}
trap 'on_error ${LINENO}' ERR

wait_for_node() {
  local node="$1"
  local timeout_s="${2:-20}"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    if ros2 node list --no-daemon 2>/dev/null | grep -qx "${node}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

lifecycle_state() {
  local node="$1"
  ros2 lifecycle get "${node}" 2>/dev/null | awk '{print $1}' || true
}

activate_lifecycle_node() {
  local node="$1"
  if ! wait_for_node "${node}" 30; then
    echo "Timed out waiting for lifecycle node ${node}" >&2
    return 1
  fi

  local state
  state="$(lifecycle_state "${node}")"
  case "${state}" in
    active)
      echo "${node} already active"
      ;;
    inactive)
      if ! ros2 lifecycle set "${node}" activate; then
        echo "Failed to activate ${node}" >&2
        return 1
      fi
      ;;
    unconfigured|"")
      if ! ros2 lifecycle set "${node}" configure; then
        echo "Failed to configure ${node}" >&2
        return 1
      fi
      if ! ros2 lifecycle set "${node}" activate; then
        echo "Failed to activate ${node}" >&2
        return 1
      fi
      ;;
    *)
      echo "Unexpected lifecycle state for ${node}: ${state}" >&2
      ros2 lifecycle get "${node}" || true
      return 1
      ;;
  esac
}

wait_for_tf() {
  local target="$1"
  local source="$2"
  local timeout_s="${3:-30}"
  local log_file="${LOG_DIR}/tf_check_${target//\//_}_${source//\//_}.log"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    timeout 3s ros2 run tf2_ros tf2_echo "${target}" "${source}" >"${log_file}" 2>&1 || true
    if grep -q "Translation:" "${log_file}"; then
      return 0
    fi
    sleep 1
  done
  cat "${log_file}" >&2 || true
  echo "Timed out waiting for TF ${target} -> ${source}" >&2
  return 1
}

stop_existing_nav2() {
  local patterns=(
    "nav2_map_server.*map_server"
    "nav2_planner.*planner_server"
    "nav2_controller.*controller_server"
    "nav2_bt_navigator.*bt_navigator"
    "static_transform_publisher.*map.*camera_init"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
}

echo "=== Go2 dock Nav2 stack ==="
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "LOG_DIR=${LOG_DIR}"
echo "MAP_YAML=${MAP_YAML}"
echo "NAV2_PARAMS=${NAV2_PARAMS}"
echo "BT_XML=${BT_XML}"
echo "ENABLE_DOCK_SAFE_GATE=${ENABLE_DOCK_SAFE_GATE}"

ros2 daemon stop 2>/dev/null || true

if [[ "${STOP_EXISTING_NAV2}" == "true" || "${STOP_EXISTING_NAV2}" == "1" ]]; then
  stop_existing_nav2
fi

wait_for_tf camera_init unitree_go2/base_footprint 45

if [[ "${START_MAP_TO_CAMERA_STATIC}" == "true" || "${START_MAP_TO_CAMERA_STATIC}" == "1" ]]; then
  start_bg map_to_camera ros2 run tf2_ros static_transform_publisher \
    "${MAP_TO_CAMERA_X}" "${MAP_TO_CAMERA_Y}" "${MAP_TO_CAMERA_Z}" \
    "${MAP_TO_CAMERA_QX}" "${MAP_TO_CAMERA_QY}" "${MAP_TO_CAMERA_QZ}" "${MAP_TO_CAMERA_QW}" \
    map camera_init
fi

start_bg map_server ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="${MAP_YAML}" \
  -p use_sim_time:=false
activate_lifecycle_node /map_server

wait_for_tf map unitree_go2/base_footprint 30

start_bg planner_server ros2 run nav2_planner planner_server --ros-args \
  -r __node:=planner_server \
  --params-file "${NAV2_PARAMS}"

start_bg controller_server ros2 run nav2_controller controller_server --ros-args \
  --params-file "${NAV2_PARAMS}" \
  -r cmd_vel:=/nav2_debug/cmd_vel

start_bg bt_navigator ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file "${NAV2_PARAMS}" \
  -p default_bt_xml_filename:="${BT_XML}" \
  -p default_nav_to_pose_bt_xml:="${BT_XML}" \
  -p default_nav_through_poses_bt_xml:="${BT_XML}"

activate_lifecycle_node /planner_server
activate_lifecycle_node /controller_server
activate_lifecycle_node /bt_navigator

if [[ "${ENABLE_DOCK_SAFE_GATE}" == "true" || "${ENABLE_DOCK_SAFE_GATE}" == "1" ]]; then
  echo "Enabling dock safety gate ..."
  ros2 service call /go2_safe/reset_estop std_srvs/srv/Trigger "{}" >/dev/null || true
  ros2 service call /go2_safe/set_enabled std_srvs/srv/SetBool "{data: true}" >/dev/null
else
  echo "Dock safety gate remains disabled. Enable manually when ready:"
  echo "  ros2 service call /go2_safe/set_enabled std_srvs/srv/SetBool \"{data: true}\""
fi

echo
echo "Dock Nav2 stack started. Keep this terminal open."
echo "Use laptop RViz to send goals. Stop motion:"
echo "  ros2 service call /go2_safe/stop std_srvs/srv/Trigger \"{}\""
echo "Logs: ${LOG_DIR}"
echo "Tail follows. Ctrl-C stops dock Nav2 and disables the safety gate."
echo

startup_complete=1
tail -n +1 -F "${LOG_DIR}"/*.log &
tail_pid="$!"
wait
