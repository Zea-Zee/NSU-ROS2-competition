import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    robot_controller = Node(
        package='autorace_core_roscomnadzor',
        executable='run',
    )

    return LaunchDescription([
        TimerAction(
                period=1.0,
                actions=[robot_controller])
    ])