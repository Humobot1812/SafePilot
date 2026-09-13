from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='drone_controller',
            executable='teleop_node',
            name='drone_teleop',
            output='screen'
        ),

    ])