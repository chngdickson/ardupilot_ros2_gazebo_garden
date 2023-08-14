#!/usr/bin/env python3
# Ros essentials
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Ros msgs
from sensor_msgs.msg import PointCloud2, Image

# Python libs
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from matplotlib import pyplot as plt
from ultralytics import YOLO
import matplotlib.pyplot as plt

class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__(self.__class__.__name__) # type: ignore
    self.sub_pc2 = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.sub_pc2_cb, 10)
    self.sub_cam = self.create_subscription(Image, '/rgbd_camera/image', self.sub_cam_cb, 10)
    self.model = YOLO("bestmodel.pt")
    
  def sub_pc2_cb(self, msg:PointCloud2):
    width, height = msg.width, msg.height
    xyz = self.pc2_to_numpy(msg)
    print(xyz.shape)
    x,y,z = xyz[:,:,0], xyz[:,:,1], xyz[:,:,2]
    r,g,b = xyz[:,:,3], xyz[:,:,4], xyz[:,:,5]
    return
  
  def sub_cam_cb(self, img:Image):
    img_np_arr = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    print(img_np_arr.shape)
    results = self.model(img_np_arr)
    boxes = []
    for r in results[0]:
      boxes.append(r.boxes.xyxy.cpu().numpy())
    plt.imshow(img_np_arr)
    plt.show()
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