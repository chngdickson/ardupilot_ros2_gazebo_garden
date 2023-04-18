import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

  pth_my_world_ros = get_package_share_directory('tutorial_1')
  pth_mavros_configs = os.path.join(pth_my_world_ros,"config","mavros_configs")
  param1 = os.path.join(pth_mavros_configs,"apm_pluginlists.yaml")
  param2 = os.path.join(pth_mavros_configs,"apm_config.yaml")
  
  mavros_node = Node(
    package='mavros',
    executable='mavros_node',
    namespace='mavros',
    output="screen",
    parameters=[
      param2,
      {
        "fcu_url":"udp://:14551@",
        "gcs_url":"udp://:14550",
        "target_system_id":1,
        "target_component_id":1,
        "fcu_protocol":"v2.0",
      }
      ]
  )
  
  
  
  def on_exit_restart(event:ProcessExited, context:LaunchContext):
    print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
      event.action.name, event.pid, event.returncode))
    if event.returncode != 0 and 'mavros_node' in event.action.name:
        return generate_launch_description() # respawn node action
        
  return LaunchDescription([
    mavros_node,
    RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart))
  ])