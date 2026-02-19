# INF_A ROS Topic Contracts (Frozen v1)

This document freezes the ROS 2 interfaces between core autonomy nodes for simulation and integration.

## Contract matrix

| Producer | Interface | Type | Consumer(s) | Notes |
|---|---|---|---|---|
| `infa_slam` | `/infa/local_pose` | `geometry_msgs/PoseStamped` | `infa_mission`, `infa_inference` | Local VIO/SLAM pose in `infa_local` frame. |
| `infa_slam` | `/infa/structure_plane` | `std_msgs/Float32MultiArray` | `infa_mission` | Plane coefficients `[a,b,c,d]`. |
| `infa_mission` | `/infa/mission/state` | `std_msgs/String` | `infa_safety` | `IDLE|LOCK|SWEEP|COMPLETE|ABORT`. |
| `infa_mission` | `/infa/mission/cmd_vel` | `geometry_msgs/Twist` | Flight controller bridge | Structure-lock guidance output. |
| `infa_mission` | `/infa/mission/sweep_waypoints` | `geometry_msgs/PoseArray` | Planner/executor | Lawnmower path waypoints. |
| `infa_mission` | `/infa/mission_complete` | `std_msgs/Bool` | `infa_sync` | `True` only when mission transitions to `COMPLETE`. |
| `infa_safety` | `/infa/mission/preempt` | `std_msgs/String` | `infa_mission` | Safety-triggered preempt reason/action. |
| `infa_safety` | `/infa/flight/fallback_mode` | `std_msgs/String` | Flight controller bridge | VIO fallback command (Position Hold optical flow). |
| `infa_safety` | `/infa/safety/avoidance_cmd` | `geometry_msgs/Twist` | Flight controller bridge | Virtual cage pushback vector. |
| `infa_safety` | `/infa/mission/safe_zone_target` | `geometry_msgs/PoseStamped` | Navigation/autoland | Safe-zone target for power abort. |
| `infa_safety` | `/infa/safety/event` | `std_msgs/String` (JSON) | Ops/telemetry pipeline | Serialized event payload. |
| `infa_inference` | `/infa/inference/events` | `std_msgs/String` (JSON) | Supabase writer / telemetry | Defect event with class, confidence, pose, image path. |
| `infa_sync` | `/infa/sync/status` | `std_msgs/String` (JSON) | Ops dashboard | Upload/poll lifecycle status. |
| `infa_sync` | `/infa/sync/reconstruction_result` | `std_msgs/String` (JSON) | GeoJSON/export pipeline | Reconstruction task result metadata. |

## Required safety inputs

These feeds are required for `infa_safety` evaluation logic:

- `/infa/vio/quality` (`std_msgs/Float32`)
- `/infa/vio/pose_age_s` (`std_msgs/Float32`)
- `/infa/battery/state` (`sensor_msgs/BatteryState`)
- `/infa/wind/load_proxy` (`std_msgs/Float32`)
- `/infa/lidar/scan_360` (`sensor_msgs/LaserScan`)

## Top-level simulation launch

Use the integrated launch file to run frozen-contract nodes together:

```bash
ros2 launch infa_mission infa_full_mission_sim.launch.py
```

This launch runs:

- `infa_slam/slam_bridge_node`
- `infa_mission/mission_node`
- `infa_inference/defect_inference_node`
- `infa_safety/safety_supervisor`
- `infa_sync/sync_node`
