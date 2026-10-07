# Unitree Go2 Navigation Experiments

ROS 2/Nav2 navigation experiments in Isaac Sim and on a real Unitree Go2 with a Livox Mid-360 and Fast-LIO2.

- **Simulation:** connected an Isaac Sim Go2 experiment to ROS 2/Nav2 for map-based navigation trials.
- **Real robot:** integrated a Mid-360, Fast-LIO2, dock-side Nav2, a gated velocity path, and Unitree SDK2 for Go2 navigation trials.

My work focuses on **system integration, configuration, debugging, mapping utilities, TF and odometry interfaces, and experimental validation**. This repository contains the experiment-layer scripts, Nav2 parameters, offline map tools, and technical records.

The simulation work builds on [Zhefan Xu's Isaac Sim Go2 ROS 2 project](https://github.com/Zhefan-Xu/isaac-go2-ros2). Its Go2 model, locomotion controller, and RL policy are not my work. The upstream implementation is not copied into this independent repository.

## Simulation pipeline

~~~text
Isaac Sim / Isaac Lab Go2
  -> simulated LiDAR, camera, ground-truth odometry and ROS 2 bridge
  -> static synthetic map + Nav2 (Navfn global planner, DWB local controller)
  -> Twist command -> upstream RL locomotion policy -> simulated Go2 joints
~~~

Simulation odometry comes from simulator state, not LiDAR localization. The Nav2 parameters, launcher, map-generation script, and synthetic map example are included here; the upstream simulator and locally adapted ROS bridge are external dependencies.

## Real-robot pipeline

~~~text
Mid-360 -> Livox ROS 2 driver -> Fast-LIO2 -> /Odometry + TF
  -> planar base_footprint projection + same-start static map transform
  -> map_server + Nav2 (Navfn / DWB)
  -> /nav2_debug/cmd_vel -> safety gate -> /go2_safe/cmd_vel
  -> localhost UDP bridge -> Unitree SDK2 SportClient.Move -> Go2
~~~

The dock ran sensing, odometry, Nav2, and motion-side processes; the laptop supplied RViz, goals, and offline map conversion. The included tools can turn a Fast-LIO PCD and recorded LiDAR/odometry into a Nav2 occupancy map. ROS 2 Humble was used on the laptop and the dock launchers target ROS 2 Foxy.

## Current Limitations

- Simulation and real robot use different sensing, localization, TF, mapping, and motion interfaces. **This is not a validated sim-to-real transfer or deployment.**
- The current real-robot Nav2 local costmap has **no live LiDAR obstacle layer**; its only plugin is inflation. The global costmap uses a static map and inflation. Real-time local LiDAR obstacle avoidance has not been demonstrated.
- The real navigation trial used a **same-start static map transform**. AMCL/global relocalization beyond that assumption has not been validated.
- The simulation map's alignment with the Isaac scene and the exact Nav2-to-bridge command topic need a reproducible check.
- The public scripts have machine-specific paths removed and real-motion defaults disabled, but **the public version has not been rerun on the robot**. A configured 0.50 m/s limit is not a measured speed.

The source workspace retains private mapping and navigation diagnostic bags. They support that ROS command and odometry streams ran during a trial, but do not establish repeatable goal success or per-command SDK acceptance. See [experiment evidence](docs/experiments.md).

## Repository guide

- [Contributions and AI-assisted development](docs/CONTRIBUTIONS.md)
- [Public technical audit](docs/project_audit.md)
- [Simulation integration and reproducibility gaps](docs/simulation_integration.md)
- [Real-robot integration contract](docs/real_robot_setup.md)
- [Experiment evidence](docs/experiments.md)
- [Source-to-public file manifest](docs/COPY_MANIFEST.md)
- [Third-party projects and required dependencies](THIRD_PARTY.md)

The simulation Nav2 entry point is scripts/start_nav2_servers.sh. The dock entry points are scripts/start_dock_wired_motion_stack_on_dock.sh and scripts/start_dock_nav2_on_dock.sh; despite the first filename, it also accepts Wi-Fi ROS discovery. These research scripts require external workspaces, a private map, and user-provided DDS configuration. Real sending defaults to dry run and the public Nav2 launcher leaves the safety gate disabled. Review TF, network, watchdog, speed, and stop behavior on the target setup before any real-motion test.

## Ongoing work

1. Add and validate a live Mid-360 obstacle source in the real local costmap.
2. Verify global localization beyond same-start static alignment.
3. Measure repeated goal completion, stopping error, SDK command acceptance, and speed with synchronized logs.
4. Reproduce the simulation map/TF alignment and record a versioned end-to-end trial.

No third-party robot model, RL weights, SDK, Fast-LIO2 source, Livox driver source, rosbag, large PCD, or real-site map is included.
