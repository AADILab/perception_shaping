from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_shaping',
            executable='driver'
        ),
        Node(
            package='perception_shaping',
            executable='scan_cleaner'
        ),
        Node(
            package='perception_shaping',
            executable='gazebo_pose_publisher'
        ),
        Node(
            package='perception_shaping',
            executable='supervisor'
        )
    ])
