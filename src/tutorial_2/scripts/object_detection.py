#!/usr/bin/env python3
# Custom Libs
from sort_hungarian_KF import Sort_kf

# Ros essentials
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
import message_filters as mf_ros2

# Ros msgs
from sensor_msgs.msg import PointCloud2, PointField, Image, JointState
from std_msgs.msg import Float64

# Python libs
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from matplotlib import pyplot as plt
import math

# YOLO
from ultralytics import YOLO
import cv2
import numpy as np
import torch
import os
import time

# Clustering
from sklearn.cluster import MiniBatchKMeans
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt

class PointCloudHelper():
  def __init__(self) -> None:
    pass
  
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

def draw_boxes(img, bbox, color="red", categories=None, names=None, offset=(0, 0)):
  img = img.copy()
  # print(bbox.shape)
  if color == "red":
    color = (0, 0, 255)
  else:
    color = (255, 255, 0)
  for i, box in enumerate(bbox):
    x1, y1, x2, y2 = [int(i) for i in box]
    x1 += offset[0]
    x2 += offset[0]
    y1 += offset[1]
    y2 += offset[1]

    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)

  # Draw Centers
  centers = np.vstack([bbox[:,0]+(bbox[:,2]-bbox[:,0])/2, bbox[:,1]+(bbox[:,3]-bbox[:,1])/2]).T
  for center in centers:
    cv2.circle(img, (int(center[0]), int(center[1])), 5, color, -1)

  # Draw the center on the center of the image
  cy_img, cx_img = img.shape[0]/2, img.shape[1]/2
  cv2.circle(img, (int(cx_img), int(cy_img)), 5, (0,255,0), -1)
  return img

class CoordinateDetector(Node):
  def __init__(self, 
    package_name:str="tutorial_2", 
    model_dir_name:str="yolov8Weights", 
    model_filename:str = "bestmodel.pt",
    show_image: bool = True
    ):
    super().__init__(self.__class__.__name__) # type: ignore
    # Comment the parameters
    """
    Parameters:
    -----------
    package_name: str
      The name of the package where the model is stored
    model_dir_name: str
      The name of the directory inside the package where the model is stored
    model_filename: str
      The name of the model file
      
    Returns:
    --------
    None
    """
    
    # Clustering and Miscellanous essentials
    self.kmeans = MiniBatchKMeans(n_clusters=2, batch_size=256, max_iter=50, n_init="auto")
    plt.ion()
    self.show_image = show_image 
    
    # PointCloud2 helper funcs
    self.pchelper = PointCloudHelper().pc2_to_numpy

    # Yolov8
    self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    pth_package = get_package_share_directory(package_name)
    file_path = os.path.join(pth_package, model_dir_name, model_filename)
    self.model = YOLO(file_path)
    
    # KF Sort
    self.kalman_sort = Sort_kf(
      additional_params=0,
      max_undetected_times= 60,
      min_hits= 2,
      iou_threshold= 0.3,
      )
    
    # Img and Depth Sub Pub
    # a. Subscriber
    self.sub_img = mf_ros2.Subscriber(self, Image, '/rgbd_camera/image')
    self.sub_pc2 = mf_ros2.Subscriber(self, PointCloud2, '/rgbd_camera/points')
    self.subImgAndPointCloud = mf_ros2.ApproximateTimeSynchronizer([self.sub_img, self.sub_pc2], 10, 0.1).registerCallback(self.cb_ImgAndPointCloud)
    # b. Publisher
    self.pub_coor = self.create_publisher(Point, '/rgbd_camera/points/coordinate', 10)
    
    
    """ONLY WORKS IN SIMULATION"""
    # Joint Sub and Pub
    self.pub_camera = self.create_publisher(Float64, '/gimbal/cmd_tilt', 10)
    self.sub_joint = self.create_subscription(JointState, '/gimbal_joint_states', self.cb_joint_state, 10)
    """YOU CAN DELETE THIS PART IF YOU ARE NOT USING SIMULATION"""
    
  
  def cb_ImgAndPointCloud(self, img_msg:Image, pc2_msg:PointCloud2):
    depth_image = self.pchelper(pc2_msg)
    image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
    
    if depth_image is not None and image is not None:
      boxes, bbox_depth_img, xyz = self.get_coordinate(image, depth_image)
      if self.show_image:
        if boxes is not None and bbox_depth_img is not None:
          if len(boxes)>0:
            image = draw_boxes(image, boxes)
            image = np.concatenate((image, bbox_depth_img), axis=1)
        else:
          image = np.concatenate((image, np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)), axis=1)
        plt.imshow(image)
        plt.show()
        plt.pause(0.001)
        plt.clf()

  def cb_joint_state(self, msg:JointState):
    idx = list(msg.name).index("tilt_joint")
    if abs(90-math.degrees(msg.position[idx])) > 1.0:
      self.pub_camera.publish(Float64(data=90.0)) 
      self._logger.info(f"Tilt Joint:  {math.degrees(msg.position[idx])}")
      
  def detect_and_track(self, img:np.ndarray)->np.ndarray:
    """
    Performs Object Detection and Tracking using Kalman Filter with Hungarian algorithm
    
    Params:
    -------
    img: np.ndarray
    
    Returns:
    --------
    pred_bbox_and_curr_bbox: np.ndarray
    """
    results = self.model.predict(img, verbose=False)
    if len(results[0])>0:
      boxes = results[0].boxes.xyxy.cpu().numpy()
      pred_bbox_and_curr_bbox = self.kalman_sort.update(boxes.reshape(-1,4))
      return pred_bbox_and_curr_bbox
    else:
      return np.array([])
  
  def filter(self, arr:np.ndarray)->np.ndarray:
    """
    Remove Nan values and replace them with mean of the array
    
    Params:
    -------
    arr: np.ndarray
    
    Returns:
    --------
    arr: np.ndarray
    """
    arr[~np.isfinite(arr)] = np.mean(arr[np.isfinite(arr)])
    return arr
  
  def get_coordinate(self, image, depth_image):
    """
    It's technically a publisher function but it's called inside the callback function
    It publishes the center point of the tree.
    
    Params:
    -------
    image: np.ndarray
    depth_image: np.ndarray
    
    Returns:
    --------
    VISUALIZATION
    boxes: np.ndarray
    bbox_depth_img: np.ndarray
    xyz: np.ndarray
    
    
    Algorithm
    1. Detect the objects and KalmanFilter track
    2. Find the object closest to the center of the image
    3. Find the depth of indexes within the object
    4. Normalize the depth values
    5. Clustering to find the Tree
    6. Publishes the Center Point to ROS2 Topic
    7. Returns Parameters for Visualization
    """
       
    # 1. Detect the objects in the image
    boxes = self.detect_and_track(image)
    h,w = image.shape[0], image.shape[1]
    cx_img, cy_img = w/2, h/2
    
    # 2. Find the object closest to the center of the image
    if boxes is not None and len(boxes)>0:
      distances_from_center = []
      centers = np.vstack([boxes[:,0]+(boxes[:,2]-boxes[:,0])/2, boxes[:,1]+(boxes[:,3]-boxes[:,1])/2]).T
      
      for i, center in enumerate(centers):
        distances_from_center.append(math.dist((cx_img, cy_img), center))
      idx_closest = np.argmin(distances_from_center)
      box_closest = boxes[idx_closest]
      
      # 3. Find the depth of indexes within the object
      box_closest = np.array(box_closest).astype(int)
      slice_y,slice_x = slice(box_closest[0], box_closest[2]), slice(box_closest[1], box_closest[3])
      
      # Filter the depth image
      depth_image = self.filter(depth_image)
      depth_image_bbox = depth_image[slice_x, slice_y,0:3]
      # For visualization
      depth_bbox_img = np.ones((h,w,3)).astype(np.uint8)*255
      depth_bbox_img[slice_x, slice_y] = depth_image_bbox

      
      # 4. Find xyz
      # a. find z
      row, col, ch = depth_image_bbox.shape
      
      def findMiddle(number:int)-> int:
        if number%2==0:
          return int(number/2)
        else:
          return int((number-1)/2)
      # 0=z, 1=x, 2=y
      y0 = depth_image_bbox[:,findMiddle(col),1].mean()
      x0 = depth_image_bbox[findMiddle(row),:,2].mean()
      z = depth_image_bbox[:,:,0]
      
      # 5. Clustering to find the only the nearest value
      # There are Ground, Tree and possibly other objects
      # This Clusters each of them and decides the nearest value is the Tree
      # The furthest value is the ground.
      z_flat = z.reshape(-1,1)
      self.kmeans.fit(z_flat)
      z = self.kmeans.cluster_centers_.min()
      xyz = (x0,y0,z)
      
      # Publish point
      p = Point()
      p.x, p.y, p.z = float(x0), float(y0), float(z)
      if math.isinf(p.x) or math.isinf(p.y) or math.isinf(p.z):
        return None, None, None
      else:
        self.pub_coor.publish(p)

      return boxes, depth_bbox_img, xyz
    else:
      return None, None, None


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
  lift_controller = CoordinateDetector()
  executor = MultiThreadedExecutor(num_threads=4)
  thread = threading.Thread(target=run_executor, args=(executor, lift_controller), daemon=True)
  thread.start()
  print("I am printing The callback")
  rclpy.spin(lift_controller)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()