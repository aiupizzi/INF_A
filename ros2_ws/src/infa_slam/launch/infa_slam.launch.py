from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='infa_slam',
            executable='slam_bridge_node',
            name='infa_slam_bridge',
            output='screen',
            parameters=[
                {
                    'publish_rate_hz': 30.0,
                    'local_pose_topic': '/infa/local_pose',
                    'structure_plane_topic': '/infa/structure_plane',
                }
            ],
        )
    ])
