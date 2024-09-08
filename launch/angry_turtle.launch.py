from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='usi_angry_turtle',
            executable='angry_turtle_controller',
            name='angry_turtle',
            output='screen'
        )
    ])
