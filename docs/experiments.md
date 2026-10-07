# Experiment evidence and limits

| Experiment | Evidence retained privately in the source workspace | What it supports | What it does not support |
| --- | --- | --- | --- |
| Mid-360 / Fast-LIO2 mapping | A 109.45 s ROS bag with 1,097 LiDAR, 21,892 IMU, 1,098 odometry and 2,196 TF messages; PCD and derived observed map | Sensor/odometry recording and offline map generation | Map ground-truth accuracy or global relocalization |
| Dock-side navigation diagnostic | A 114.71 s bag with 1,052 Nav2 velocity, 2,295 gated velocity/status and 1,150 odometry messages | Nav2, gate and odometry streams operated during that run | Per-command SDK acceptance, measured robot speed or repeatable goal completion |
| Isaac Sim Nav2 | Launcher/configuration and cumulative local Nav2 logs | The simulation Nav2 path was attempted and produced successful-goal log lines | A versioned, reproducible current-config success rate |

The real-site map, rosbag, PCD and raw robot logs are private. They may reveal physical layout, device details or operator behavior. This public repository contains only a synthetic simulation map. No table entry is a dynamic-obstacle-avoidance result.

The included map-generation script was run in an isolated temporary directory during repository preparation. Its generated PGM matched the included synthetic PGM byte for byte. This checks file reproducibility, not map-to-Isaac-scene alignment.
