from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='infa_slam',
            executable='slam_bridge_node',
            name='infa_slam_bridge',
            output='screen',
        ),
        Node(
            package='infa_mission',
            executable='mission_node',
            name='infa_mission_node',
            output='screen',
            parameters=[
                {
                    'standoff_m': 2.0,
                    'lane_spacing_m': 0.6,
                    'speed_mps': 0.4,
                    'overlap_ratio': 0.2,
                    'abort_distance_error_m': 1.0,
                    'patch_width_m': 5.0,
                    'patch_height_m': 3.0,
                }
            ],
        ),
    ])
