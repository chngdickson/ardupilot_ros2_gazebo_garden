#!/usr/bin/env python3
# Ros essentials
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Ros msgs
from sensor_msgs.msg import PointCloud2, PointField, Image

# Python libs
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from matplotlib import pyplot as plt
import subprocess
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
  def __init__(self):
    super().__init__(self.__class__.__name__) # type: ignore
    # PointCloud2 helper funcs
    self.pchelper = PointCloudHelper().pc2_to_numpy

    # Yolov8
    self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    dir_path , model_weight_file_name = os.path.dirname(os.path.realpath(__file__)), "bestmodel.pt"
    file_path = os.path.join(dir_path, model_weight_file_name)
    self.model = YOLO(file_path)
    
    # Subscribers
    self.sub_pc2 = self.create_subscription(PointCloud2, '/rgbd_camera/points', self.sub_pc2_cb, 10)
    self.sub_img = self.create_subscription(Image, '/rgbd_camera/image', self.sub_img_cb, 10)
    self.pub_coor = self.create_publisher(Point, '/rgbd_camera/points/coordinate', 10)
    
    # Clustering = 
    print("hello")
    self.kmeans = MiniBatchKMeans(n_clusters=2, batch_size=256, max_iter=50, n_init="auto")
    # Coordinate detector
    self.image = None
    self.depth_image = None
    while True:
      if self.image is not None and self.depth_image is not None:
        image = self.image
        start = time.time()
        boxes, bbox_depth_img, xyz = self.get_coordinate()
        end = time.time()
        print('Total time is',end-start)
        if boxes is not None:
          if len(boxes)>0:
            image = draw_boxes(image, boxes)
            image = np.concatenate((image, bbox_depth_img), axis=1)
        cv2.imshow("image", image)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
      rclpy.spin_once(self, timeout_sec=0.01)
    
  def sub_pc2_cb(self, msg:PointCloud2) -> None:
    # pose = subprocess.Popen(["gz topic -e  -t world/iris_runway2/pose/info -n1 | grep \"base_link\" -A6"], shell=True, stdout=subprocess.PIPE)
    # pose_xyz = pose.communicate()[0].decode("utf-8")
    width, height = msg.width, msg.height
    xyz = self.pchelper(msg)
    self.depth_image = xyz
    # print(xyz.shape)
    
    x,y,z = xyz[:,:,0], xyz[:,:,1], xyz[:,:,2]
    r,g,b = xyz[:,:,3], xyz[:,:,4], xyz[:,:,5]
    return
  
  def sub_img_cb(self, msg:Image) -> None:
    self.image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    # print(self.image.shape)
    return
  
  def detect_and_track(self, img:np.ndarray):
    results = self.model.predict(img, verbose=False)
    if len(results[0])>0:
      boxes = results[0].boxes.xyxy.cpu().numpy()
      return boxes
  
  def get_coordinate(self):
    """
    Algorithm
    1. Get the image and depth image
    2. Detect the objects in the image
    3. Find the object closest to the center of the image
    4. Find the depth of indexes within the object
    5. Normalize the depth values
    """
    
    # 1. Get the image and depth image
    if self.image is not None and self.depth_image is not None:
      image, depth_image = self.image, self.depth_image
      
      # 2. Detect the objects in the image
      boxes = self.detect_and_track(image)
      h,w = image.shape[0], image.shape[1]
      cx_img, cy_img = w/2, h/2
      # 3. Find the object closest to the center of the image
      if boxes is not None:
        distances_from_center = []
        centers = np.vstack([boxes[:,0]+(boxes[:,2]-boxes[:,0])/2, boxes[:,1]+(boxes[:,3]-boxes[:,1])/2]).T
        
        for i, center in enumerate(centers):
          distances_from_center.append(math.dist((cx_img, cy_img), center))
        idx_closest = np.argmin(distances_from_center)
        box_closest = boxes[idx_closest]
        
        # 4. Find the depth of indexes within the object
        box_closest = np.array(box_closest).astype(int)
        slice_y,slice_x = slice(box_closest[0], box_closest[2]), slice(box_closest[1], box_closest[3])
        depth_image_bbox = depth_image[slice_x, slice_y,0:3]
        # For visualization
        depth_bbox_img = np.ones((h,w,3)).astype(np.uint8)*255
        depth_bbox_img[slice_x, slice_y] = depth_image_bbox

        
        # 5. Find xyz
        # a. find z
        row, col, ch = depth_image_bbox.shape
        
        def findMiddle(number:int)-> int:
          if number%2==0:
            return int(number/2)
          else:
            return int((number-1)/2)
        # 0=z, 1=x, 2=y
        x0 = depth_image_bbox[:,findMiddle(col),1].mean()
        x1 = depth_image_bbox[findMiddle(row),:,1].mean()
        y0 = depth_image_bbox[findMiddle(row),:,2].mean()
        y1 = depth_image_bbox[:,findMiddle(col),2].mean()
        z = depth_image_bbox[:,:,0]
        
        # Handle nans and flatten it, then cluster it
        z_no_non = z[~np.isnan(z)]
        z_flat = z_no_non[~np.isinf(z_no_non)].reshape(-1,1)
        self.kmeans.fit(z_flat)
        z = self.kmeans.cluster_centers_.min()
        # print(x0,x1,y0,y1,z)
        xyz = (x0,y0,z)
        
        # Publish point
        p = Point()
        p.x, p.y, p.z = float(x0), float(y0), float(z)
        self.pub_coor.publish(p)

        # print(x,y,z)
        # depth_bbox_img = cv2.cvtColor(depth_bbox_img,cv2.COLOR_GRAY2RGB)

        return boxes, depth_bbox_img, xyz
      else:
        return None, None, None
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