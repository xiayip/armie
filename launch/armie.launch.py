"""Four-mode game; HMI/expression nodes are not part of this launch."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config=LaunchConfiguration('config')
    runtime='/workspaces/zephyr-dev/zephyr_ws/.armie-runtime/bin/python'
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=os.path.join(get_package_share_directory('armie'),'config','game.yaml')),
        DeclareLaunchArgument('motion_backend',default_value='mock'),
        DeclareLaunchArgument('motion_enabled',default_value='false'),
        DeclareLaunchArgument('senses_backend',default_value='mock'),
        DeclareLaunchArgument('panel',default_value='false'),
        DeclareLaunchArgument('grasp_planning',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare('zephyr_arm_moveit_config'),
            '/launch/move_group.launch.py']),condition=IfCondition(LaunchConfiguration('grasp_planning'))),
        DeclareLaunchArgument('senses_python',default_value=runtime if os.path.isfile(runtime) else ''),
        Node(package='armie',executable='game',name='armie_game',output='screen',parameters=[config,{
            'motion_backend':LaunchConfiguration('motion_backend'),
            'motion_enabled':ParameterValue(LaunchConfiguration('motion_enabled'),value_type=bool)}]),
        Node(package='armie',executable='senses',name='armie_senses',output='screen',
             prefix=LaunchConfiguration('senses_python'), additional_env={'MPLCONFIGDIR':'/tmp/armie-mpl'},
             parameters=[config,{'senses_backend':LaunchConfiguration('senses_backend')}]),
        Node(package='armie',executable='panel',name='armie_panel',output='screen',condition=IfCondition(LaunchConfiguration('panel'))),
    ])
