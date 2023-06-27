import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

  pth_my_world_ros = get_package_share_directory('tutorial_2')

  # # Camera launch
  cam_bridge_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pth_my_world_ros, 'launch', 'camera_bridge.launch.py')
      )
  )
  # RGBD camera launch
  cam_rgbd_bridge_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pth_my_world_ros, 'launch', 'camera_rgbd_bridge.launch.py')
      )
  )
  # Bridge launch
  gazebo_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pth_my_world_ros, 'launch', 'gazebo.launch.py')
      )
  )
  
  # tf bridge
  tf_bridge_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pth_my_world_ros, 'launch', 'joints_bridge.launch.py')
      )
  )
  
  mavros_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pth_my_world_ros, 'launch', 'mavros_node.launch.py')
      )
  )
  cmd = ["sim_vehicle.py","-v","ArduCopter","-f","gazebo-iris","--model","JSON","--map","--console","--custom-location","2.868877,101.411648,0,0"]
  # # cmd = ["sim_vehicle.py","-v","ArduCopter","-f","gazebo-iris","--model","JSON","--console","--custom-location","2.868877,101.411648,0,0"]
  subprocess.Popen(cmd)
  
  
  # return LaunchDescription([first_launch_file, second_launch_file])
  return LaunchDescription([
    gazebo_launch_file,
    # cam_bridge_launch_file,
    cam_rgbd_bridge_launch_file,
    tf_bridge_launch_file
    #   mavros_node
  ])