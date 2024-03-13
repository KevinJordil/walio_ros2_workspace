import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('kinova_gen3_6dof_robotiq_2f_85_moveit_config'),
                'launch/robot.launch.py')
            ),
            launch_arguments={
                "robot_ip": "192.168.2.10",
                "use_fake_hardware": "True"
            }.items()
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            remappings=[
                ("/cmd_vel", "/twist_controller/commands")
            ]
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/controller_manager/switch_controller',
                'controller_manager_msgs/srv/SwitchController',
                '{start_controllers: [twist_controller], stop_controllers: [joint_trajectory_controller], strictness: 1, start_asap: true}'
            ],
            output='screen'
        )
    ])
