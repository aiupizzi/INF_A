import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ZedMiniPipeline:
    """Lightweight stand-in for a ZED Mini VIO/SLAM frontend."""

    def __init__(self) -> None:
        self._tick = 0

    def get_local_pose(self) -> Tuple[float, float, float, float]:
        """Returns x, y, z, yaw in local frame."""
        self._tick += 1
        t = self._tick * 0.02
        x = 0.2 * math.sin(t)
        y = 0.15 * math.cos(t * 0.7)
        z = 1.5 + 0.05 * math.sin(t * 0.5)
        yaw = 0.1 * math.sin(t * 0.3)
        return x, y, z, yaw

    def estimate_structure_plane(self) -> Tuple[float, float, float, float]:
        """Plane in local frame as ax + by + cz + d = 0."""
        # Mock wall at x = 5 m with normal toward -x: -1*x + 0*y + 0*z + 5 = 0
        return -1.0, 0.0, 0.0, 5.0


class SlamBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('infa_slam_bridge')
        self.declare_parameter('local_pose_topic', '/infa/local_pose')
        self.declare_parameter('structure_plane_topic', '/infa/structure_plane')
        self.declare_parameter('publish_rate_hz', 30.0)

        local_pose_topic = self.get_parameter('local_pose_topic').value
        structure_plane_topic = self.get_parameter('structure_plane_topic').value
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pipeline = ZedMiniPipeline()
        self.pose_pub = self.create_publisher(PoseStamped, local_pose_topic, 10)
        self.plane_pub = self.create_publisher(Float32MultiArray, structure_plane_topic, 10)

        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._publish_state)
        self.get_logger().info('infa_slam bridge started')

    def _publish_state(self) -> None:
        x, y, z, yaw = self.pipeline.get_local_pose()
        a, b, c, d = self.pipeline.estimate_structure_plane()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'infa_local'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose_msg)

        plane_msg = Float32MultiArray()
        plane_msg.data = [a, b, c, d]
        self.plane_pub.publish(plane_msg)


def main() -> None:
    rclpy.init()
    node = SlamBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
