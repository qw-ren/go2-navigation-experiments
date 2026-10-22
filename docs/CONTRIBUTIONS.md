> 2026-10-22：下文为迁移前实验记录，保留历史边界。最新云仓储仿真、实际AMCL、独立LIO/LIVO和实时障碍验证见[升级说明](CLOUD_NAVIGATION_UPGRADE.md)与[当前模块](../isaac_sim/go2_warehouse/README.md)；旧真机结果不因仿真升级而自动获得验证。

# Contributions and provenance

## Built / integrated by me in this project

- Assembled a dock-side Go2 navigation experiment around Mid-360, Fast-LIO2, Nav2, a software velocity gate, a local UDP bridge, and a separate Unitree SDK2 worker.
- Built and debugged the ROS topic, TF, map, and command contracts between those existing systems. A diagnostic bag confirms that Nav2 command, gated command, and odometry messages were present in one real-robot trial.
- Added offline tools that combine Fast-LIO PCD occupancy evidence with LiDAR-ray/free-space evidence from a recorded bag to produce a Nav2 occupancy map.
- Ran simulation-side Nav2 integration experiments on the upstream Isaac Sim Go2 project, then compared the interfaces with the real robot. These are separate experimental paths.

These statements describe system-level project work and local artifacts. They do not claim that every code line was handwritten or that every branch was tested on a robot.

## Configured/adapted from existing frameworks

- Navfn and DWB through ROS 2 Nav2 parameter files; behavior-tree-based navigation without recovery actions in the real dock trial.
- Isaac Sim / Isaac Lab Go2 ROS 2 integration inherited from the upstream simulator. The local source fork changed its bridge, TF, and ROS wiring, but this repository does not redistribute that upstream-derived implementation.
- Fast-LIO2, Livox, CycloneDDS, and Unitree SDK2 were configured or called through local launch scripts. Their algorithms and libraries were not developed here.

## Third-party systems

The robot model, RL policy and checkpoints, Isaac Sim, ROS 2/Nav2, Mid-360 driver, Fast-LIO2, and Unitree SDK2 are third-party work. See [THIRD_PARTY.md](../THIRD_PARTY.md) for links.

## AI-assisted implementation

Codex assisted with generating, editing, and debugging parts of this experiment code and documentation. Git history does not provide reliable line-by-line authorship for the local files. The contribution claimed here is the experimental architecture, integration decisions, debugging, validation, and ability to explain the resulting system, not that every line was typed manually.

## Still need to understand / verify

- Confirm the exact Fast-LIO2 and Livox configuration/versions on the dock, including topic and time-sync settings.
- Verify simulator-map alignment, its ROS command topic and bridge timing under a reproducible run.
- Validate TF calibration, same-start localization drift, stopping behavior, and SDK command return values with independent measurements.
- Add and test a live obstacle source in the real local costmap before claiming real-time LiDAR-based avoidance.
- Measure repeated goal completion and actual speed. The 0.50 m/s value in a local parameter file is a configured limit, not a measured result.
