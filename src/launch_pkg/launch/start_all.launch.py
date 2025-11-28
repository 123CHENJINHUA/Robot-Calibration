import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import sys

def generate_launch_description():
    
    action_node_data_collection = launch_ros.actions.Node(
        package='calibration_pkg',
        executable='data_collection',
        name='data_collection',
        output='screen'
    )

    action_node_remote_control = launch_ros.actions.Node(
        package='remote_control_pkg',
        executable='remote_control_node',
        name='remote_control_node',
        output='screen'
    )

    return launch.LaunchDescription([
        action_node_data_collection,
        action_node_remote_control,
    ])
