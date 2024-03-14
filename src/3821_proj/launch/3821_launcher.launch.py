from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='3821_proj',
            executable='3821_planner',
            output='screen',
        ),
        Node(
            package='3821_proj',
            executable='3821_follower',
            output='screen',
        ),
    ])