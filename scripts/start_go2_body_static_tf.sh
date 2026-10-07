#!/usr/bin/env bash
set -euo pipefail

# Manufacturer matrix:
#   T_go2_body_mid360 =
#     R_y(+13 deg), t=(0.1870, 0.0, 0.0803)
#
# Fast-LIO2 publishes:
#   camera_init -> body
#
# To avoid giving `body` a second parent, publish the inverse transform:
#   body -> unitree_go2/base_link

ros2 run tf2_ros static_transform_publisher \
  -0.164143635 0.0 -0.120307775 \
  0.0 -0.113203214 0.0 0.993571856 \
  body unitree_go2/base_link
