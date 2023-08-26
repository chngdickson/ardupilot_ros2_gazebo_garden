#!/usr/bin/env python3 
import sys
import rclpy
import threading
from typing import List
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node

from ardu_helper import Ardu_Ros_Connect

from geometry_msgs.msg import Point
def run_executors(nodes: List[Node], lambda_func):
  
  try:
    def executor_wrapper(executor, nodes: List[Node]):
      for node in nodes:
        executor.add_node(node)
      try:
        executor.spin()
      finally:
        executor.shutdown()
        for node in nodes:
          node.destroy_node()
    
    executor = MultiThreadedExecutor()

    # Spin in Separate thread to prevent blocking
    executor_thread = threading.Thread(target=executor_wrapper, args=(executor,nodes), daemon=True)
    executor_thread.start()
    # Perform other functions
    
    lambda_func()
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(1)
  finally:
    # Shut down ROS2
    rclpy.try_shutdown()
      

def main(node_drone: Ardu_Ros_Connect):
  TAKEOFF_ALT = 12.0
  node_drone.wait4connect()
  node_drone.state_GUIDED()
  node_drone.wait4start()
  node_drone.arm_disarm(arming=True)
  node_drone.arm_disarm(arming=False)
  
  node_drone.await_waypoints_before_takeoff()
  # node_drone.clr_waypoints()
  wps = node_drone.get_waypoints()
  node_drone.arm_disarm(arming=True)
  node_drone.takeoff(TAKEOFF_ALT)
  
  for i, wp in enumerate(wps):
    node_drone.go_destination_global_lla(
      lat=wp[0], long=wp[1], alt=wp[2]+TAKEOFF_ALT, heading=wp[3]
    )
    node_drone.get_logger().info(f"wp [{i}] reached out of {len(wps)}")
    
    node_drone.PID_obj_det(
      tol= Point(x=0.12, y=0.12, z=3.5),
      kp = [0.3, 0.3, 0.3],
      ki = [0.1, 0.1, 0.1],
      kd = [0.2, 0.2, 0.2],
      timeout= 1.0
    )
    node_drone.get_logger().info(f"PID [{i}] completed, moving to next wp")
  node_drone.returnToHome()
  
  
if __name__ == '__main__':
  args = sys.argv
  rclpy.init(args=args)
  node_drone = Ardu_Ros_Connect()
  run_executors([node_drone], main(node_drone))