# Source-to-public file manifest

Source means the local isaac-go2-ros2 experiment workspace at the audit date. Only the files listed here were brought into this independent repository. A path absent from the table was not implicitly copied.

| Source path | Public path | Publication handling |
| --- | --- | --- |
| scripts/start_dock_wired_motion_stack_on_dock.sh | same | Paths parameterized; DDS configuration required externally; historical filename retained |
| scripts/start_dock_nav2_on_dock.sh | same | Private map required; repo-local Nav2 config/BT; auto-enabling gate disabled by default |
| scripts/start_dock_fastlio_on_dock.sh | same | External workspace paths parameterized; DDS configuration required externally |
| scripts/start_go2_body_static_tf.sh | same | Copied as experiment-layer extrinsic publisher |
| scripts/project_fastlio_odom_to_base_footprint.py | same | Copied; ROS odometry-to-planar-TF projector |
| scripts/safe_cmd_vel_gate.py | same | Copied; ROS speed limiter/watchdog/service gate |
| scripts/go2_safe_cmd_vel_udp_bridge.py | same | Copied; ROS Twist to local UDP |
| scripts/go2_unitree_sport_udp_worker.py | same | Copied; external Unitree SDK2 import remains a dependency |
| scripts/start_go2_safe_cmd_vel_udp_bridge_on_dock.sh | same | Private DDS configuration required externally |
| scripts/start_go2_unitree_sport_udp_worker_on_dock.sh | same | SDK Python path made optional and externally supplied |
| scripts/pcd_to_nav2_map.py | same | Copied; PCD parser and map writer |
| scripts/bag_pcd_to_nav2_map.py | same | Copied; requires ROS bag and Livox message packages |
| scripts/start_nav2_servers.sh | same | Laptop-specific paths removed; stale-process cleanup disabled by default |
| generate_map.py | scripts/generate_sim_map.py | Moved and output directed to maps/simulation; terrain parameters still manually mirrored |
| nav2/nav2_params.yaml | same | Copied simulation parameters |
| nav2/nav2_params_real_fastlio_foxy.yaml | same | Copied dock parameters; behavior-tree path made repo-relative |
| nav2/real_nav_to_pose_no_recovery.xml | same | Copied no-recovery behavior tree |
| map/sim_map.yaml, map/sim_map.pgm | maps/simulation/sim_map.yaml, maps/simulation/sim_map.pgm | Synthetic example only; scene alignment unverified |

The source audit was rewritten as docs/project_audit.md. It is not a verbatim copy: personal paths, network information, and private data references were removed.

Not copied: any upstream Go2 model/controller/whole ROS bridge file or RL checkpoint; Isaac Sim installer; Unitree SDK; Fast-LIO2/Livox source; CycloneDDS network XML; real-site maps, rosbag, PCD, logs, cached output, private notes, or third-party reference documents. The source repository remains unchanged except for its origin remote URL.
