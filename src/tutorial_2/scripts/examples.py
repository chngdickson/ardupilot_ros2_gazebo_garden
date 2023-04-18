import rclpy
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
    super().__init__('helper')
    self.subscription = self.create_subscription(sensor_msgs.msg.JointState, 'joint_states', self.listener_callback, 10)
          
  def listener_callback(self, msg):
    new_msg = sensor_msgs.msg.JointState()
  
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
  rclpy.init(args=args)
  
  # Create the node
  lift_controller = MinimalPublisher()
  print("I am printing The callback")
  rclpy.spin(lift_controller)
  
if __name__ == '__main__':
  main()