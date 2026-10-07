# Public project audit

Evidence labels: **CONFIRMED** means supported by source, Git objects or local recorded artifacts; **INFERRED** means plausible but not independently replayed; **UNKNOWN** means the available records cannot establish it.

## Upstream and local scope

**CONFIRMED:** The source workspace was a fork of [Zhefan Xu's Isaac Sim Go2 ROS 2 project](https://github.com/Zhefan-Xu/isaac-go2-ros2), branch isaacsim-4.5, at commit f201692. At audit time, local HEAD equaled the local upstream branch ref. The work lived in staged, unstaged, and untracked files, not in a set of local commits.

**CONFIRMED:** Unmodified upstream files included the Go2 environment definition, terrain generator, RL checkpoints, and demo media. Eight upstream-tracked files had local changes: cfg/sim.yaml, env/sim_env.py, go2/go2_ctrl.py, go2/go2_ctrl_cfg.py, go2/go2_sensors.py, isaac_go2_ros2.py, ros2/go2_ros2_bridge.py, and rviz/go2.rviz. None of those whole upstream-derived files is copied here.

**CONFIRMED:** The copied experiment layer consists of locally added launch scripts, Nav2 configuration, TF projection, safety gate, UDP/SDK worker, offline map tools, and a synthetic map example. Source-to-public-file details are in [COPY_MANIFEST.md](COPY_MANIFEST.md). **UNKNOWN:** local Git history cannot establish which lines were handwritten, AI-generated, or adapted from examples.

## Simulation pipeline

**CONFIRMED:** The upstream Isaac Sim/Isaac Lab Go2 model and RL policy provided the simulated robot and locomotion. A local ROS bridge modification exposed simulator-state odometry, simulated RTX LiDAR and camera data, TF, and command subscription. The local Nav2 configuration used a synthetic map, Navfn global planning, DWB local control and a point-cloud obstacle layer.

~~~text
Isaac Sim Go2 + simulated sensors / ground-truth odometry
  -> ROS 2 bridge -> static synthetic map + Nav2 -> Twist -> upstream RL policy -> Go2 joints
~~~

**UNKNOWN:** The generated map's exact match to the Isaac scene and the final command topic across Nav2 versions need a fresh run. No simulation-side Fast-LIO2 localization was established in this path.

## Real-robot pipeline

**CONFIRMED:** Mid-360 driver topics /livox/lidar and /livox/imu fed an externally installed Fast-LIO2 on the dock. Fast-LIO2 supplied /Odometry and a camera_init to body transform. Local scripts published body to base_link and a planar camera_init to base_footprint transform. A static map to camera_init edge supplied same-start alignment. The dock ran map_server, Navfn, DWB and BT navigator. The command chain used /nav2_debug/cmd_vel, a velocity gate, /go2_safe/cmd_vel, a localhost UDP bridge and Unitree SDK2 SportClient.Move.

~~~text
Mid-360 -> Livox driver -> Fast-LIO2 -> odometry / TF
  -> same-start map + dock Nav2 -> safety gate -> UDP bridge -> Unitree SDK2 -> Go2
~~~

**CONFIRMED:** The dock Nav2 local costmap has only inflation; the global costmap uses static map and inflation. Neither has a live LiDAR obstacle source. **UNKNOWN:** vendor-side avoidance and SDK per-command acceptance were not established by the recorded ROS bag. Global relocalization beyond the same-start assumption remains unvalidated.

## Simulation versus real robot

| Component | Simulation | Real robot | Relationship |
| --- | --- | --- | --- |
| Robot interface | IsaacLab Go2 / RL policy | Unitree SportClient | Different |
| LiDAR | Simulated RTX sensor | Livox Mid-360 | Different |
| Odometry | Simulator root state | Fast-LIO2 estimate | Different |
| Localization | Static map alignment + truth state | Static same-start map alignment + Fast-LIO2 | Different |
| Mapping | Synthetic terrain-derived map | PCD + recorded rays to occupancy map | Different |
| TF | map to odom to base_link | map to camera_init to body/footprint | Different |
| Global planner | Navfn | Navfn, separate ROS distro/config | Conceptually same |
| Local controller | DWB | DWB, separate ROS distro/config | Conceptually same |
| Costmap | Static map + simulated cloud obstacle layer | Static map; no live obstacle layer | Different |
| Command interface | ROS Twist to RL policy | ROS Twist to gate/UDP/SDK | Different |

**INFERRED:** These are related navigation experiments, not a validated sim-to-real migration. The common parts are ROS 2/Nav2 concepts and planner/controller plugin families, not the sensing, localization, map, or motion implementation.

## Confirmed contributions and limits

**CONFIRMED:** The local project has experiment-layer configuration, launch/bridge/safety tools and an offline map workflow. The source workspace held a mapping bag, map product and a navigation diagnostic bag, which are intentionally not redistributed here.

**INFERRED:** Field observations support that the robot moved under navigation goals, but neither long-term success rate nor precise final-goal error is established by this public copy. **UNKNOWN:** whether the public-path launchers behave identically on the robot; they have not been rerun after sanitization.

**CONFIRMED:** The configured 0.50 m/s forward limit is not proof of actual robot speed. No live LiDAR obstacle layer means real-time Nav2 local LiDAR avoidance is not a supported claim.

## Questions to resolve next

1. What exact installed Fast-LIO2, Livox, Unitree SDK2 and Nav2 versions produced the recorded run?
2. Which process owns every TF edge at runtime, and how stable is same-start map alignment?
3. Which topic actually carries simulation Nav2 velocity into the upstream bridge?
4. What did the SDK return for velocity calls during a navigation goal?
5. How will a live obstacle layer and independent stopping test be validated before a stronger avoidance claim?
