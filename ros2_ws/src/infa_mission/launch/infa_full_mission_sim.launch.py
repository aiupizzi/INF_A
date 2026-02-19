from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='infa_slam',
                executable='slam_bridge_node',
                name='infa_slam_bridge',
                output='screen',
                parameters=[
                    {
                        'publish_rate_hz': 20.0,
                        'local_pose_topic': '/infa/local_pose',
                        'structure_plane_topic': '/infa/structure_plane',
                    }
                ],
            ),
            Node(
                package='infa_mission',
                executable='mission_node',
                name='infa_mission_node',
                output='screen',
                parameters=[
                    {
                        'pose_topic': '/infa/local_pose',
                        'plane_topic': '/infa/structure_plane',
                        'state_topic': '/infa/mission/state',
                        'cmd_topic': '/infa/mission/cmd_vel',
                        'waypoint_topic': '/infa/mission/sweep_waypoints',
                        'complete_topic': '/infa/mission_complete',
                        'preempt_topic': '/infa/mission/preempt',
                        'auto_start': True,
                    }
                ],
            ),
            Node(
                package='infa_inference',
                executable='defect_inference_node',
                name='infa_defect_inference',
                output='screen',
                parameters=[
                    {
                        'pose_topic': '/infa/local_pose',
                        'event_topic': '/infa/inference/events',
                    }
                ],
            ),
            Node(
                package='infa_safety',
                executable='safety_supervisor',
                name='infa_safety_supervisor',
                output='screen',
                parameters=[
                    {
                        'mission_state_topic': '/infa/mission/state',
                        'preempt_topic': '/infa/mission/preempt',
                    }
                ],
            ),
            Node(
                package='infa_sync',
                executable='sync_node',
                name='infa_sync',
                output='screen',
                parameters=[
                    {
                        'mission_complete_topic': '/infa/mission_complete',
                    }
                ],
            ),
        ]
    )
