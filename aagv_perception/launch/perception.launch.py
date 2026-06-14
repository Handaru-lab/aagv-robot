from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aagv_perception',
            executable='ball_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'target_frame': 'base_link',
                'hsv_lower': [5, 120, 80],
                'hsv_upper': [20, 255, 255],
                'min_radius_px': 5.0,
            }],
        ),
    ])
