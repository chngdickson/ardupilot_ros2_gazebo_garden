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
    # Topic in Gazebo Directly becomes to topic name in ROS2.
    # E.g. in rgb_camera.sdf, Topic rgb_camera/Image 
    # Now becomes rgb_camera/Image in ros2.
    # Bridge
    modelname = "iris_ardu_camera"
    cam_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgb_camera/Image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
        remappings=[
            ('/rgb_camera/Image','/camera')
        ],
        output='screen'
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/default/model/{modelname}/joint_state@'
            'sensor_msgs/msg/JointState@gz.msgs.Model',
            f'/model/{modelname}/pose@gz.msgs.Pose'
            'tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        remappings=[
            (f'/world/default/model/{modelname}/joint_state', '/joint_states')
            ],
        output='screen'
    )
    
    joint_ctrl_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gimbal/cmd_roll@std_msgs/msg/Float64@gz.msgs.Double',
            '/gimbal/cmd_tilt@std_msgs/msg/Float64@gz.msgs.Double'],
        output='screen'
    )
    # ros2 topic pub /gimbal/cmd_roll std_msgs/msg/Float64 '{data: -0.7804}'


    return LaunchDescription([
        cam_bridge,
        tf_bridge, 
        joint_ctrl_bridge
    ])