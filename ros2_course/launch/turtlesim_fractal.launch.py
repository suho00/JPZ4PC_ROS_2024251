from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Turtlesim alap node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # A te vezérlőd
        Node(
            package='ros2_course',
            executable='turtlesim_controller',
            name='turtlesim_controller',
            output='screen'
        )
    ])
