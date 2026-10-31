from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('armie'), 'config', 'armie.yaml')),
        DeclareLaunchArgument('motion_backend', default_value='mock'),
        DeclareLaunchArgument('motion_enabled', default_value='false'),
        DeclareLaunchArgument('vision_backend', default_value='mock'),
        DeclareLaunchArgument('hmi_transport', default_value='mock'),
        Node(package='armie', executable='pet', name='armie_pet', output='screen',
             parameters=[config, {
                 'motion_backend': LaunchConfiguration('motion_backend'),
                 'motion_enabled': ParameterValue(LaunchConfiguration('motion_enabled'), value_type=bool)}]),
        Node(package='armie', executable='vision', name='armie_vision', output='screen',
             parameters=[config, {'vision_backend': LaunchConfiguration('vision_backend')}]),
        Node(package='armie', executable='hmi', name='armie_hmi', output='screen',
             parameters=[config, {'hmi_transport': LaunchConfiguration('hmi_transport')}]),
    ])
