from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencn_pkg',
            executable='opencn_node',
            name='opencn_node'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=['./launch/joy_config.yaml']
        ),
        Node(
            package='motion_control',
            executable='motion_control_node',
            name='motion_control_node'
        ),
    ])
