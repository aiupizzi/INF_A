# infa_sync

ROS 2 package that waits for mission completion, uploads mission imagery to WebODM, and monitors reconstruction status.

## Behavior

1. Subscribes to mission complete signal (`std_msgs/Bool`, default `/infa/mission_complete`).
2. Collects images from `imagery_dir` matching `image_pattern`.
3. Authenticates to WebODM API (`/api/token-auth/`) and creates a task under `webodm_project_id`.
4. Polls task state until success/failure/timeout.
5. Publishes JSON status messages on `~/sync_status` and result payload on `~/reconstruction_result`.

## Run

```bash
ros2 run infa_sync sync_node --ros-args --params-file config/example_sync.yaml
```
