from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from .mission_logic import LawnmowerConfig, LawnmowerPlanner, StructureLockConfig, StructureLockController


class MissionState(str, Enum):
    IDLE = 'IDLE'
    LOCK = 'LOCK'
    SWEEP = 'SWEEP'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('infa_mission_node')

        self.declare_parameter('standoff_m', 2.0)
        self.declare_parameter('lane_spacing_m', 0.6)
        self.declare_parameter('speed_mps', 0.4)
        self.declare_parameter('overlap_ratio', 0.2)
        self.declare_parameter('abort_distance_error_m', 1.0)
        self.declare_parameter('patch_width_m', 5.0)
        self.declare_parameter('patch_height_m', 3.0)

        self.controller = StructureLockController(
            StructureLockConfig(
                standoff_m=float(self.get_parameter('standoff_m').value),
                speed_mps=float(self.get_parameter('speed_mps').value),
                abort_distance_error_m=float(self.get_parameter('abort_distance_error_m').value),
            )
        )
        self.planner = LawnmowerPlanner(
            LawnmowerConfig(
                lane_spacing_m=float(self.get_parameter('lane_spacing_m').value),
                overlap_ratio=float(self.get_parameter('overlap_ratio').value),
                speed_mps=float(self.get_parameter('speed_mps').value),
            )
        )

        self.state = MissionState.IDLE
        self.pose: Optional[Tuple[float, float, float]] = None
        self.plane: Optional[List[float]] = None
        self.sweep_waypoints: List[Tuple[float, float]] = []

        self.pose_sub = self.create_subscription(PoseStamped, '/infa/local_pose', self._on_pose, 10)
        self.plane_sub = self.create_subscription(Float32MultiArray, '/infa/structure_plane', self._on_plane, 10)

        self.state_pub = self.create_publisher(String, '/infa/mission/state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/infa/mission/cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(PoseArray, '/infa/mission/sweep_waypoints', 10)

        self.start_srv = self.create_service(Trigger, '/infa/mission/start', self._start_cb)
        self.abort_srv = self.create_service(Trigger, '/infa/mission/abort', self._abort_cb)

        self.timer = self.create_timer(0.1, self._tick)
        self._publish_state()

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_plane(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self.plane = list(msg.data[:4])

    def _start_cb(self, _, response: Trigger.Response) -> Trigger.Response:
        if self.state not in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT}:
            response.success = False
            response.message = f'Cannot start from {self.state.value}'
            return response
        self.state = MissionState.LOCK
        self._publish_state()
        response.success = True
        response.message = 'Mission started'
        return response

    def _abort_cb(self, _, response: Trigger.Response) -> Trigger.Response:
        self.state = MissionState.ABORT
        self._publish_state()
        response.success = True
        response.message = 'Mission aborted'
        return response

    def _tick(self) -> None:
        if self.state in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT}:
            return
        if self.pose is None or self.plane is None:
            return

        if self.state == MissionState.LOCK:
            try:
                vx, vy = self.controller.compute_command(self.pose, self.plane)
            except RuntimeError as exc:
                self.state = MissionState.ABORT
                self.get_logger().error(str(exc))
                self._publish_state()
                return

            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            self.cmd_pub.publish(cmd)

            if abs(vx) < 0.02 and abs(vy) < 0.02:
                self._plan_sweep()
                self.state = MissionState.SWEEP
                self._publish_state()

        elif self.state == MissionState.SWEEP:
            # Placeholder: real mission executor would track/consume waypoints.
            self.state = MissionState.COMPLETE
            self._publish_state()

    def _plan_sweep(self) -> None:
        width = float(self.get_parameter('patch_width_m').value)
        height = float(self.get_parameter('patch_height_m').value)
        self.sweep_waypoints = self.planner.generate_patch_waypoints(width, height)

        waypoint_msg = PoseArray()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'infa_local'

        for u, v in self.sweep_waypoints:
            pose = PoseStamped().pose
            pose.position.x = u
            pose.position.y = v
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            waypoint_msg.poses.append(pose)

        self.waypoint_pub.publish(waypoint_msg)

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        self.get_logger().info(f'Mission state: {msg.data}')


def main() -> None:
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
