from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .webodm_client import WebODMClient


class MissionSyncNode(Node):
    """Sync mission imagery to WebODM when mission completion is announced."""

    # WebODM status values: 40=completed, 50=failed/canceled in current API variants.
    TERMINAL_SUCCESS = {40}
    TERMINAL_FAILURE = {50}

    def __init__(self) -> None:
        super().__init__('infa_sync')

        self.declare_parameter('mission_complete_topic', '/infa/mission_complete')
        self.declare_parameter('imagery_dir', '/tmp/infa/imagery')
        self.declare_parameter('image_pattern', '*.jpg')
        self.declare_parameter('webodm_url', 'http://127.0.0.1:8000')
        self.declare_parameter('webodm_username', 'admin')
        self.declare_parameter('webodm_password', 'admin')
        self.declare_parameter('webodm_project_id', 1)
        self.declare_parameter('poll_interval_s', 10.0)
        self.declare_parameter('poll_timeout_s', 3600.0)
        self.declare_parameter('verify_tls', True)
        self.declare_parameter('task_name_prefix', 'infa-mission')
        self.declare_parameter('task_options_json', '{}')

        topic = self.get_parameter('mission_complete_topic').value
        self._status_pub = self.create_publisher(String, '~/sync_status', 10)
        self._result_pub = self.create_publisher(String, '~/reconstruction_result', 10)
        self._sub = self.create_subscription(Bool, topic, self._on_mission_complete, 10)

        self.get_logger().info(f'Listening for mission completion on: {topic}')

    def _publish_status(self, status: str, **metadata: object) -> None:
        payload = {'status': status, **metadata}
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _on_mission_complete(self, msg: Bool) -> None:
        if not msg.data:
            return

        try:
            task_result = self._upload_and_poll()
            out = {
                'task_id': task_result['task_id'],
                'status': task_result['status'],
                'task': task_result['task'],
            }
            result_msg = String()
            result_msg.data = json.dumps(out)
            self._result_pub.publish(result_msg)
            self._publish_status('completed', task_id=task_result['task_id'])
            self.get_logger().info(f"WebODM reconstruction complete for task {task_result['task_id']}")
        except Exception as exc:  # noqa: BLE001 - bubble failure into ROS logs/status.
            self._publish_status('failed', error=str(exc))
            self.get_logger().error(f'Mission sync failed: {exc}')

    def _upload_and_poll(self) -> dict[str, object]:
        imagery_dir = Path(self.get_parameter('imagery_dir').value)
        pattern = self.get_parameter('image_pattern').value
        image_paths = sorted(imagery_dir.glob(pattern))
        if not image_paths:
            raise FileNotFoundError(f'No images found in {imagery_dir} matching {pattern}')

        self._publish_status('uploading', image_count=len(image_paths))
        self.get_logger().info(f'Preparing upload of {len(image_paths)} images from {imagery_dir}')

        client = WebODMClient(
            base_url=str(self.get_parameter('webodm_url').value),
            username=str(self.get_parameter('webodm_username').value),
            password=str(self.get_parameter('webodm_password').value),
            verify_tls=bool(self.get_parameter('verify_tls').value),
        )
        options = json.loads(self.get_parameter('task_options_json').value)
        task_name = f"{self.get_parameter('task_name_prefix').value}-{int(time.time())}"
        project_id = int(self.get_parameter('webodm_project_id').value)

        task_id = client.create_task(
            project_id=project_id,
            image_paths=image_paths,
            task_name=task_name,
            options=options,
        )
        self.get_logger().info(f'Uploaded imagery to WebODM task {task_id}')

        poll_interval = float(self.get_parameter('poll_interval_s').value)
        timeout_s = float(self.get_parameter('poll_timeout_s').value)
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            task = client.get_task(project_id, task_id)
            self._publish_status('processing', task_id=task_id, webodm_status=task.status)
            if task.status in self.TERMINAL_SUCCESS:
                return {'task_id': task_id, 'status': 'completed', 'task': task.output}
            if task.status in self.TERMINAL_FAILURE:
                raise RuntimeError(f'WebODM task {task_id} finished in failure state {task.status}')
            time.sleep(poll_interval)

        raise TimeoutError(f'Timed out waiting for WebODM task {task_id} after {timeout_s:.0f}s')


def main() -> None:
    rclpy.init()
    node = MissionSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
