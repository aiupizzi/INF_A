import json

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Float32, String

from infa_safety.mission_state_machine import MissionStateMachine, SafetyEvent, SafetyEventType
from infa_safety.monitors import PowerAbortMonitor, VioHealthMonitor, VirtualCageAvoidance


class SafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__('safety_supervisor')

        self.declare_parameter('safe_zone_frame', 'map')
        self.declare_parameter('safe_zone_x', 0.0)
        self.declare_parameter('safe_zone_y', 0.0)
        self.declare_parameter('safe_zone_z', 2.0)

        self.declare_parameter('event_topic', '/infa/safety/event')
        self.declare_parameter('fallback_topic', '/infa/flight/fallback_mode')
        self.declare_parameter('preempt_topic', '/infa/mission/preempt')
        self.declare_parameter('safe_zone_target_topic', '/infa/mission/safe_zone_target')
        self.declare_parameter('avoidance_topic', '/infa/safety/avoidance_cmd')
        self.declare_parameter('vio_quality_topic', '/infa/vio/quality')
        self.declare_parameter('vio_pose_age_topic', '/infa/vio/pose_age_s')
        self.declare_parameter('battery_topic', '/infa/battery/state')
        self.declare_parameter('wind_proxy_topic', '/infa/wind/load_proxy')
        self.declare_parameter('lidar_topic', '/infa/lidar/scan_360')
        self.declare_parameter('mission_state_topic', '/infa/mission/state')

        self._event_seq = 0
        self._state_machine = MissionStateMachine()

        self._vio_monitor = VioHealthMonitor()
        self._power_monitor = PowerAbortMonitor()
        self._cage_monitor = VirtualCageAvoidance()

        self._event_pub = self.create_publisher(String, str(self.get_parameter('event_topic').value), 10)
        self._fallback_pub = self.create_publisher(String, str(self.get_parameter('fallback_topic').value), 10)
        self._preempt_pub = self.create_publisher(String, str(self.get_parameter('preempt_topic').value), 10)
        self._safe_zone_pub = self.create_publisher(PoseStamped, str(self.get_parameter('safe_zone_target_topic').value), 10)
        self._avoidance_pub = self.create_publisher(Twist, str(self.get_parameter('avoidance_topic').value), 10)

        self.create_subscription(Float32, str(self.get_parameter('vio_quality_topic').value), self._on_vio_quality, 10)
        self.create_subscription(Float32, str(self.get_parameter('vio_pose_age_topic').value), self._on_vio_pose_age, 10)
        self.create_subscription(BatteryState, str(self.get_parameter('battery_topic').value), self._on_battery_state, 10)
        self.create_subscription(Float32, str(self.get_parameter('wind_proxy_topic').value), self._on_wind_proxy, 10)
        self.create_subscription(LaserScan, str(self.get_parameter('lidar_topic').value), self._on_lidar_scan, 10)
        self.create_subscription(String, str(self.get_parameter('mission_state_topic').value), self._on_mission_state, 10)

        self.create_timer(0.1, self._evaluate_safety)

    def _on_vio_quality(self, msg: Float32) -> None:
        self._vio_monitor.update_quality(msg.data)

    def _on_vio_pose_age(self, msg: Float32) -> None:
        self._vio_monitor.update_pose_age(msg.data)

    def _on_battery_state(self, msg: BatteryState) -> None:
        self._power_monitor.update_battery(msg.voltage, msg.current)

    def _on_wind_proxy(self, msg: Float32) -> None:
        self._power_monitor.update_wind_load_proxy(msg.data)

    def _on_lidar_scan(self, msg: LaserScan) -> None:
        ranges = [r if r > 0.01 else msg.range_max for r in msg.ranges]
        self._cage_monitor.update_lidar(ranges, msg.angle_min, msg.angle_increment)

    def _on_mission_state(self, msg: String) -> None:
        self._state_machine.update_mission_phase(msg.data)

    def _evaluate_safety(self) -> None:
        for event_dict in [self._vio_monitor.evaluate(), self._power_monitor.evaluate(), self._cage_monitor.evaluate()]:
            if event_dict is None:
                continue
            self._event_seq += 1
            event = SafetyEvent(
                event_type=event_dict['event_type'],
                reason=event_dict['reason'],
                action=event_dict['action'],
                seq=self._event_seq,
            )
            self._handle_event(event, event_dict)

    def _handle_event(self, event: SafetyEvent, payload: dict) -> None:
        preempted = self._state_machine.preempt_with_safety(event)
        if not preempted:
            return

        self.get_logger().warning(payload['operator_alert'])

        event_msg = String()
        event_msg.data = json.dumps(
            {
                'event_type': event.event_type.value,
                'reason': event.reason,
                'action': event.action,
                'mission_phase': self._state_machine.current_phase.value,
            }
        )
        self._event_pub.publish(event_msg)

        if event.event_type in (SafetyEventType.VIO_DEGRADED, SafetyEventType.VIO_LOST):
            fallback = String()
            fallback.data = 'POSITION_HOLD_OPTICAL_FLOW'
            self._fallback_pub.publish(fallback)

        if event.event_type == SafetyEventType.POWER_ABORT:
            self._publish_safe_zone()

        if event.event_type == SafetyEventType.VIRTUAL_CAGE_PUSHBACK:
            self._publish_avoidance(payload)

        preempt_msg = String()
        preempt_msg.data = f'{event.event_type.value}:{event.action}'
        self._preempt_pub.publish(preempt_msg)

    def _publish_safe_zone(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('safe_zone_frame').value)
        msg.pose.position.x = float(self.get_parameter('safe_zone_x').value)
        msg.pose.position.y = float(self.get_parameter('safe_zone_y').value)
        msg.pose.position.z = float(self.get_parameter('safe_zone_z').value)
        msg.pose.orientation.w = 1.0
        self._safe_zone_pub.publish(msg)

    def _publish_avoidance(self, payload: dict) -> None:
        vec = payload.get('pushback_vector', {'x': 0.0, 'y': 0.0})
        msg = Twist()
        msg.linear.x = float(vec.get('x', 0.0))
        msg.linear.y = float(vec.get('y', 0.0))
        msg.linear.z = float(payload.get('velocity_limit_mps', 0.0))
        self._avoidance_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
