#!/usr/bin/env bash
set -euo pipefail

# Run on the Go2 dock.
#
# Preconditions:
# - MID-360 driver is already running on the dock and publishing /livox/lidar
#   and /livox/imu.
# - Fast-LIO2 has been built for the dock ROS 2 Foxy environment.
#
# This keeps raw LiDAR/IMU traffic local to the dock. The laptop should start
# scripts/start_real_fastlio_localization_viz.sh with START_FASTLIO=false, so
# it only consumes /Odometry and TF from this dock-side Fast-LIO instance.

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
GO2_WS_SETUP="${GO2_WS_SETUP:-${HOME}/go2_ws/install/setup.bash}"
FASTLIO_SETUP="${FASTLIO_SETUP:-${HOME}/ws_fastlio/install/setup.bash}"
FASTLIO_WORKDIR="${FASTLIO_WORKDIR:-${HOME}}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-mid360.yaml}"
GO2_LINK_MODE="${GO2_LINK_MODE:-wifi}"
: "${CYCLONEDDS_URI:?Set CYCLONEDDS_URI to a private local CycloneDDS XML file URI}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY

for setup_file in "${ROS_SETUP}" "${GO2_WS_SETUP}" "${FASTLIO_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing setup file: ${setup_file}" >&2
    echo "Build or install the required workspace on the Go2 dock first." >&2
    exit 2
  fi
done

# ROS setup files may reference unset variables internally.
set +u
source "${ROS_SETUP}"
source "${GO2_WS_SETUP}"
source "${FASTLIO_SETUP}"
set -u

echo "=== Dock Fast-LIO2 ==="
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "FASTLIO_SETUP=${FASTLIO_SETUP}"
echo "FASTLIO_CONFIG=${FASTLIO_CONFIG}"
echo "FASTLIO_WORKDIR=${FASTLIO_WORKDIR}"

ros2 daemon stop 2>/dev/null || true

echo "Checking required packages ..."
ros2 pkg prefix livox_ros_driver2 >/dev/null
ros2 pkg prefix fast_lio >/dev/null

echo "Checking local MID-360 topics ..."
topic_list="$(ros2 topic list --no-daemon 2>/dev/null || true)"
if ! grep -qx "/livox/lidar" <<<"${topic_list}" || ! grep -qx "/livox/imu" <<<"${topic_list}"; then
  echo "Could not see /livox/lidar and /livox/imu." >&2
  echo "Start the MID-360 driver on the dock before starting dock Fast-LIO." >&2
  exit 3
fi
grep -E '^/livox/(lidar|imu)$' <<<"${topic_list}"

echo "Starting Fast-LIO2 on dock ..."
cd "${FASTLIO_WORKDIR}"
exec ros2 launch fast_lio mapping.launch.py \
  config_file:="${FASTLIO_CONFIG}" \
  use_sim_time:=false \
  rviz:=false
