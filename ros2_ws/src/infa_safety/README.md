# infa_safety

ROS 2 Python package providing `SafetySupervisor` node with deterministic mission preemption.

## Submodules

- `VioHealthMonitor`: detects VIO degraded/lost states, commands optical-flow Position Hold fallback, and emits operator alert events.
- `PowerAbortMonitor`: tracks battery voltage/current trends, estimates effective internal resistance and wind-adjusted thrust margin, and triggers abort routing to Safe Zone.
- `VirtualCageAvoidance`: consumes 360 LiDAR scan, enforces 1.0 m virtual cage pushback vector, and limits commanded velocity.

## Interfaces

### Subscriptions
- `/infa/vio/quality` (`std_msgs/Float32`)
- `/infa/vio/pose_age_s` (`std_msgs/Float32`)
- `/infa/battery/state` (`sensor_msgs/BatteryState`)
- `/infa/wind/load_proxy` (`std_msgs/Float32`)
- `/infa/lidar/scan_360` (`sensor_msgs/LaserScan`)
- `/infa/mission/state` (`std_msgs/String`)

### Publications
- `/infa/safety/event` (`std_msgs/String`, JSON payload)
- `/infa/flight/fallback_mode` (`std_msgs/String`)
- `/infa/mission/preempt` (`std_msgs/String`)
- `/infa/mission/safe_zone_target` (`geometry_msgs/PoseStamped`)
- `/infa/safety/avoidance_cmd` (`geometry_msgs/Twist`)
