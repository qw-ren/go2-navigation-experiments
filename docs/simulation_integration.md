# Simulation integration notes

The independent repository does not contain the upstream Go2 simulator, model, RL checkpoints, or the upstream-derived bridge file. Use the [isaacsim-4.5 branch of the original project](https://github.com/Zhefan-Xu/isaac-go2-ros2) with Isaac Sim/Isaac Lab and ROS 2 Humble as the simulation base.

Local experiments changed the original bridge to: accept a Twist velocity topic, publish simulator-state odometry in an odom frame, add a static map-to-odom transform, and publish odometry/pose on a timer. The simulator entry point was also adjusted to accept Hydra overrides and to hand ROS spinning to the bridge. Those upstream-derived whole files are deliberately absent here. Reproduction therefore requires reviewing and reapplying those small interface changes in the upstream checkout; a supported patch has not yet been published.

This repository supplies:

- nav2/nav2_params.yaml: the simulation Navfn/DWB and costmap configuration;
- scripts/start_nav2_servers.sh: an adapted ROS 2 Humble Nav2 launcher;
- scripts/generate_sim_map.py and maps/simulation/: a synthetic map experiment.

The map generator mirrors terrain parameters manually rather than reading the live Isaac terrain. Treat the sample map as an experiment artifact until scene geometry and map coordinates are checked together. The script runs in the laptop environment; it is not part of real-robot mapping.

**Reproduction status:** the original local setup produced cumulative Nav2 logs containing successful goals, but the public-path launcher and map alignment have not been rerun as a versioned end-to-end test.
