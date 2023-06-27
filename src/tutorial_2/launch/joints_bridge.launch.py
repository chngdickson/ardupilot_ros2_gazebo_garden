import os
import subprocess
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  tf_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    # topic@ros_type@gazebo_type
    arguments=[
        '/world/iris_runway2/model/iris_ardu_camera/joint_state@'
        'sensor_msgs/msg/JointState[gz.msgs.Model',
        '/model/iris_ardu_camera/pose@'
        'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    ],
    remappings=[
        (f'/world/iris_runway2/model/iris_ardu_camera/joint_state', '/joint_states'),
        (f'/model/iris_ardu_camera/pose', '/tf')
        ],
    output='screen'
    )
  # pose = subprocess.Popen(["gz topic -e  -t world/iris_runway2/pose/info -n1 | grep \"iris_runway2\" -A6"], shell=True, stdout=subprocess.PIPE)
  # pose_xyz = pose.communicate()[0].decode("utf-8")
  
  return LaunchDescription([
    tf_bridge
  ])