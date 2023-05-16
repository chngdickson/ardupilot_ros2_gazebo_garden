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
from launch_ros.actions import Node, node


def generate_launch_description():

  pth_my_world_ros = get_package_share_directory('tutorial_2')
  pth_mavros_configs = os.path.join(pth_my_world_ros,"config","mavros_configs")
  pth_param1 = os.path.join(pth_mavros_configs,"apm_pluginlists.yaml")
  pth_param2 = os.path.join(pth_mavros_configs,"apm_config.yaml")
  
  config_param1 = LaunchConfiguration('config_dir',default=pth_param1)
  config_param2 = LaunchConfiguration('config_dir',default=pth_param2)
  mavros_node = Node(
    package='mavros',
    executable='mavros_node',
    namespace='mavros',
    output="screen",
    parameters=[
      {
        "fcu_url":"udp://:14551@",
        "gcs_url":"udp://:115200@", # QGC use [14550], MissionPlanner [115200]
        "target_system_id":1,
        "target_component_id":1,
        "fcu_protocol":"v2.0",
        "use_sim_time":False,
        "heartbeat_rate":0.0,
        "mavros/conn/heartbeat_rate":0.0
      },
      pth_param2,
      pth_param1,
      ]
  )
  
  # helper_node = Node(
  #   package='tutorial_2',
  #   executable='helper.py',
  #   output="screen"
  # )
  
# ros2 run mavros mavros_node --ros-args --params-file src/tutorial_2/config/mavros_configs/apm_config.yaml
  
  
  def on_exit_restart(event:ProcessExited, context:LaunchContext):
    print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
      event.action.name, event.pid, event.returncode))
    if event.returncode != 0 and 'mavros_node' in event.action.name:
        return generate_launch_description() # respawn node action
        
  return LaunchDescription([
    mavros_node,
    # helper_node,
    RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart))
  ])