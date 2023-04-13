#!/usr/bin/env python3 
 
# Test code for controlling the lift mechanism on a warehouse robot
# Author: Addison Sears-Collins
# Website: https://automaticaddison.com
 
# ROS Client Library for Python
import rclpy
  
# Handles the creation of nodes
from rclpy.node import Node

import std_msgs.msg
import sensor_msgs.msg 
from std_msgs.msg import String, Float64
from mavros_msgs.msg import ActuatorControl, RCOut
from sensor_msgs.msg import JointState
import numpy as np
  
class MinimalSubscriber(Node):
  """
  Create a LiftController class, which is a subclass of the Node class.
  """
  def __init__(self):
    super().__init__('gimbal_controller')
    self.subscription = self.create_subscription(sensor_msgs.msg.JointState, 'joint_states', self.listener_callback, 10)
          
  def listener_callback(self, msg):
    """
    Callback function.
    """
    print("I am printing The callback")
    # Create a JointStates message
    new_msg = sensor_msgs.msg.JointState()
  
    # Set the message's data
    new_msg.header.stamp = self.get_clock().now().to_msg()
    new_msg.name = msg.name
    new_msg.position = msg.position
    new_msg.position[2] = 0.30
      

class MinimalPublisher(Node):
  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(Float64, '/gimbal/cmd_roll', 10)
    self.publisher2_ = self.create_publisher(Float64, '/gimbal/cmd_tilt', 10)
    timer_period = 0.00000001  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0

  def timer_callback(self):
    msg = Float64()
    msg.data = 3.1
    self.publisher_.publish(msg)
    self.publisher2_.publish(msg)
    # self.get_logger().info('Publishing: "%s"' % msg)
    self.i += 1

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  lift_controller = MinimalPublisher()
  print("I am printing The callback")
  # Spin the node so the callback function is called.
  rclpy.spin(lift_controller)
  print("Am i working?")
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  # lift_controller.destroy_node()
  
  # Shutdown the ROS client library for Python
  # rclpy.shutdown()
  
if __name__ == '__main__':
  main()