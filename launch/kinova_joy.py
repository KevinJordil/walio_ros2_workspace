from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arguments for kortex_bringup launch file
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.10',
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='gen3_lite',
            description='Type of the robot'
        ),
        DeclareLaunchArgument(
            'dof',
            default_value='6',
            description='Degrees of freedom of the robot'
        ),

        # Launching kortex_bringup
        Node(
            package='kortex_bringup',
            executable='gen3.launch.py',
            name='kortex_bringup',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_type': LaunchConfiguration('robot_type'),
                'dof': LaunchConfiguration('dof')
            }]
        ),

        # Timer to delay service call until the controllers are ready
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="Switching controllers..."),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='switch_controller_service',
                    output='screen',
                    arguments=[
                        '--ros-args',
                        '--param', 'service_name:=/controller_manager/switch_controller',
                        '--param', 'activate_controllers:=["twist_controller"]',
                        '--param', 'deactivate_controllers:=["joint_trajectory_controller"]',
                        '--param', 'strictness:=1',
                        '--param', 'activate_asap:=true'
                    ]
                )
            ]
        ),

        # Launching teleop_twist_joy
        Node(
            package='teleop_twist_joy',
            executable='teleop-launch.py',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'joy_config': 'xbox',
                'config_filepath': './launch/teleop.yaml'
            }]
        )
    ])
