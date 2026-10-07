# Third-party projects

This repository does not vendor the projects below. Their source, binaries, models, policies, and licenses remain with their respective maintainers. Check the version and license of each dependency before deploying or redistributing anything built with it.

| Project | Role in these experiments | Source |
| --- | --- | --- |
| Zhefan Xu, Isaac Sim Unitree Go2 ROS 2 | Simulation base, Go2 model setup, ROS bridge foundation, locomotion policy integration | https://github.com/Zhefan-Xu/isaac-go2-ros2 |
| NVIDIA Isaac Sim / Isaac Lab | Simulator and robot asset framework | https://docs.isaacsim.omniverse.nvidia.com/ and https://github.com/isaac-sim/IsaacLab |
| ROS 2 Navigation2 | Map server, Navfn global planner, DWB local controller, BT navigator and costmaps | https://github.com/ros-navigation/navigation2 |
| Livox ROS Driver 2 | Mid-360 LiDAR and IMU ROS messages | https://github.com/Livox-SDK/livox_ros_driver2 |
| HKU-MARS FAST-LIO | LiDAR-inertial odometry and map source | https://github.com/hku-mars/FAST_LIO |
| Unitree SDK2 Python | Go2 SportClient motion interface | https://github.com/unitreerobotics/unitree_sdk2_python |

The exact installed commits of Fast-LIO2, Livox driver, and Unitree SDK2 on the dock were not captured in this public repository. The original simulation fork has the upstream history; this independent repository intentionally does not copy its robot model, RL checkpoint, or bridge implementation.
