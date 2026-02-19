# INF_A

ROS 2 workspace additions for INF-A mission autonomy:

- `ros2_ws/src/infa_slam`: mock ZED Mini VIO/SLAM bridge publishing:
  - `/infa/local_pose` (`geometry_msgs/PoseStamped`)
  - `/infa/structure_plane` (`std_msgs/Float32MultiArray`, `[a,b,c,d]`)
- `ros2_ws/src/infa_mission`: mission logic with:
  - `StructureLockController` for 2.0 m standoff lock
  - `LawnmowerPlanner` for bounded patch sweeps
  - mission FSM (`IDLE -> LOCK -> SWEEP -> COMPLETE/ABORT`) via topics/services

## Launch

```bash
ros2 launch infa_slam infa_slam.launch.py
ros2 launch infa_mission infa_mission.launch.py
```

## Validation script

```bash
python3 ros2_ws/src/infa_mission/scripts/validate_mission_sim.py
```

The script validates standoff lock error and sweep coverage on a mock wall.
