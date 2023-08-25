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
import subprocess

def launch_setup(context, *args, **kwargs):

  pth_mavros_launcher = get_package_share_directory("tutorial_2")
  pth_param0 = pth_mavros_launcher + "/config/mavros_configs/sim_params.yaml"
  pth_param1 = pth_mavros_launcher + "/config/mavros_configs/apm_config.yaml"
  pth_param2 = pth_mavros_launcher + "/config/mavros_configs/apm_pluginlists.yaml"  
  if os.path.exists(pth_param0) == False:
    raise FileNotFoundError("File {} not found".format(pth_param0))
  log_level = LaunchConfiguration("log_level")
  
  mavros_node = Node(
    package="mavros",
    executable="mavros_node",
    output="screen",
    parameters=[pth_param0, pth_param1, pth_param2],
    arguments=["--ros-args", "--log-level", log_level]
    
  )
  cmd = ["sim_vehicle.py","-v","ArduCopter","-f","gazebo-iris","--model","JSON","--map","--console","--custom-location","2.868877,101.411648,0,0"]
  subprocess.Popen(cmd)
  return [mavros_node]
  
def generate_launch_description():
  def on_exit_restart(event:ProcessExited, context:LaunchContext):
    print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
      event.action.name, event.pid, event.returncode))
    if event.returncode != 0 and "mavros_node" in event.action.name:
      return generate_launch_description() # respawn node action
      
  return LaunchDescription([
    DeclareLaunchArgument("log_level", default_value="info"),
    OpaqueFunction(function = launch_setup)
    ])