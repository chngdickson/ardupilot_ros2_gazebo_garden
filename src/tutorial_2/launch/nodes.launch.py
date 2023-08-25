import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node



def launch_setup(context, *args, **kwargs):
  log_level = LaunchConfiguration("log_level")
  
  ardupilot_node = Node(
    package="tutorial_2",
    executable="main.py",
    output="screen",
    parameters=[],
    arguments=["--ros-args", "--log-level", log_level]
  )
  obj_det_node = Node(
    package="tutorial_2",
    executable="object_detection.py",
    output="screen",
    parameters=[],
    arguments=["--ros-args", "--log-level", log_level]
  )
  return [ardupilot_node,obj_det_node]
  
def generate_launch_description():
  def on_exit_restart(event:ProcessExited, context:LaunchContext):
    print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
      event.action.name, event.pid, event.returncode))
    if event.returncode != 0 :
      return generate_launch_description() # respawn node action
      
  return LaunchDescription([
    DeclareLaunchArgument("log_level", default_value="info"),
    OpaqueFunction(function = launch_setup)
    ])