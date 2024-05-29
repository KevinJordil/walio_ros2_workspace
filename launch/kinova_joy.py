import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    kortex_bringup_share = FindPackageShare('kortex_bringup').find('kortex_bringup')
    teleop_twist_joy_config = os.path.join(
        os.getcwd(), 'launch', 'teleop.yaml'
    )

    teleop_launch_file = os.path.join(os.getcwd(), 'launch', 'teleop-launch.py')

    
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
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                kortex_bringup_share, '/launch/gen3.launch.py'
            ]),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_type': LaunchConfiguration('robot_type'),
                'dof': LaunchConfiguration('dof')
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file)
        ),
        
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="Switching controllers..."),
                ExecuteProcess(
                    cmd=[
                        'ros2 service call ',
                        '/controller_manager/switch_controller ',
                        'controller_manager_msgs/srv/SwitchController ', 
                        '"{activate_controllers: [twist_controller], deactivate_controllers: [joint_trajectory_controller], strictness: 1, activate_asap: true}"'
                    ],
                    shell=True
                ),
            ]
        )
        
    ])
