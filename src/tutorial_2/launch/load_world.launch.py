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
    """
    FIND REQUIRED PATHS
    """
    # Gazebo Ros pth
    pth_gazebo_ros = get_package_share_directory('ros_gz_sim') 

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pth_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
           'gz_args': '-v4 -r iris_runway2.sdf'
        }.items(),
    )
    

    return LaunchDescription([
        gz_sim
    ])