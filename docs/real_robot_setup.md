# Real-robot integration contract

These scripts represent a dock-side experiment on ROS 2 Foxy with CycloneDDS and use_sim_time=false. This public-path copy has **not** been rerun on the robot. The laptop visualization side used ROS 2 Humble. The Mid-360 driver, Fast-LIO2, Nav2, and Unitree SDK2 must be installed separately; their packages are not in this repository.

## Inputs and private configuration

- CYCLONEDDS_URI: URI of a private CycloneDDS XML file for the selected network. No IP address, peer list or interface-specific XML is published here.
- GO2_WS_SETUP and FASTLIO_SETUP: setup scripts for the dock's installed driver and Fast-LIO2 workspaces.
- FASTLIO_CONFIG: the installed Mid-360 Fast-LIO configuration name or file. Its actual experiment-time contents were not captured here.
- MAP_YAML: a private Nav2 occupancy-map YAML on the dock. The real-site map is excluded from this repository.
- UNITREE_SDK_PYTHONPATH and, where required by the SDK installation, CYCLONEDDS_HOME: local Unitree SDK2 installation paths.
- UNITREE_IFACE: dock-to-Go2 device interface; default is eth0. This is distinct from the laptop-to-dock ROS 2 discovery link.

The intended startup order is: Mid-360 driver and Fast-LIO2, TF projector and safety chain via the motion-stack script; then the dock Nav2 script; finally RViz/goal input on the laptop. The public motion-stack script defaults to dry run (GO2_REAL_SEND=0). The public Nav2 script defaults to leaving the safety gate disabled. Real motion requires deliberate review and explicit local configuration after independent validation.

## ROS interface contract

| Producer | Output | Consumer |
| --- | --- | --- |
| Livox driver | /livox/lidar, /livox/imu | Fast-LIO2 |
| Fast-LIO2 | /Odometry; camera_init to body | TF projector; Nav2 |
| TF scripts | body to unitree_go2/base_link; camera_init to unitree_go2/base_footprint | Nav2 / RViz |
| Nav2 script | static map to camera_init; /map; /nav2_debug/cmd_vel | Nav2 and safety gate |
| Safe gate | /go2_safe/cmd_vel; /go2_safe/status; enable/stop/estop services | UDP bridge / operator |
| UDP bridge | local JSON datagrams; /go2_safe/udp_bridge_status | Unitree SDK worker / diagnostics |
| Unitree SDK worker | SportClient.Move(vx, 0, vyaw) | Go2 |

Only one publisher should own map to camera_init. The same-start static transform is not global localization; an AMCL or other localization experiment would need to replace it. The current dock Nav2 local costmap does not consume live LiDAR points. Do not describe the recorded run as verified real-time LiDAR local avoidance.

The original experiment included a private map and machine-specific workspace paths. Publication changed those defaults and requires a fresh dry-run check before any robot trial.
