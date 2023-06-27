#!/usr/bin/env python3
# Ros essentials
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Ros msgs
from sensor_msgs.msg import PointCloud2, PointField

# Python libs
import struct
import ctypes
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from matplotlib import pyplot as plt
import subprocess
  
class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__(self.__class__.__name__) # type: ignore
    self.sub_pc2 = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.sub_pc2_cb, 10)

  def sub_pc2_cb(self, msg:PointCloud2):

    pose = subprocess.Popen(["gz topic -e  -t world/iris_runway2/pose/info -n1 | grep \"base_link\" -A6"], shell=True, stdout=subprocess.PIPE)
    pose_xyz = pose.communicate()[0].decode("utf-8")
    print("pose_xyz",pose_xyz)
    width, height = msg.width, msg.height
    print(msg.width,msg.height)
    print("hello am i worthing")
    # arr = pc2.read_points_numpy(msg)
    # arr = np.reshape(arr,(height, width))
    # print(arr.shape)
    xyz = self.pc2_to_numpy(msg)
    print(xyz.shape)
    
    x,y,z = xyz[:,:,0], xyz[:,:,1], xyz[:,:,2]
    r,g,b = xyz[:,:,3], xyz[:,:,4], xyz[:,:,5]

    return
  
  def pc2_to_numpy(self, ros_point_cloud:PointCloud2, squeeze=True):
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = pc2.dtype_from_fields(ros_point_cloud.fields, ros_point_cloud.point_step)
    
    # parse the cloud into an array
    cloud_arr = np.frombuffer(ros_point_cloud.data, dtype_list)
    
    # Choose whether you want a 1D array or a 2D array representing the point cloud
    if squeeze and ros_point_cloud.height == 1:
      cloud_arr= np.reshape(cloud_arr, (ros_point_cloud.width,))
    else:
      cloud_arr= np.reshape(cloud_arr, (ros_point_cloud.height, ros_point_cloud.width))

    # Convert From 2D to 3D
    return cloud_arr.view(dtype=np.float32).reshape(cloud_arr.shape + (-1,))




def main(args=None):
  rclpy.init(args=args)
  def run_executor(executor, node):
    executor.add_node(node)
    try:
      executor.spin()
    finally:
      node.destroy_node()
      executor.shutdown()
  
  # Create the node
  lift_controller = MinimalSubscriber()
  executor = MultiThreadedExecutor(num_threads=2)
  thread = threading.Thread(target=run_executor, args=(executor, lift_controller), daemon=True)
  thread.start()
  print("I am printing The callback")
  rclpy.spin(lift_controller)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()