import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pth_my_world_ros = get_package_share_directory('my_world')

    # Camera launch
    cam_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pth_my_world_ros, 'launch', 'camera.launch.py')
        )
    )

    # Bridge launch
    bridge_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(pth_my_world_ros, 'launch', 'bridge.launch.py')
        )
    )

    # return LaunchDescription([first_launch_file, second_launch_file])
    return LaunchDescription([
        cam_launch_file,
        bridge_launch_file
    ])