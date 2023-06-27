#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client, SrvTypeRequest

from mavros_msgs.msg import State, GlobalPositionTarget, WaypointList, Waypoint

from scripts.helper_funcs.ardu_helper import *
from scripts.helper_funcs.geo_helper import *
import copy
import os
import pickle
import json
import subprocess
import curses

class Palm_tree(Node):
  def __init__(self):
    super().__init__(self.__class__.__name__)
    directory = "/home/ds/Documents/gh/ardugzros2/src/tutorial_2/geoids/"
    
    self.drone = Ardu_Ros_Connect()
    self.drone.wait4connect()
    self.drone.arm_disarm(arming=True)
    self.drone.initialize_local_and_global_frame()
      
  
  def get_coords(self):
    self.drone.await_waypoints_before_takeoff()
    self.wps = self.drone.get_waypoints()
    self.initial_gps_pose = copy.deepcopy(self.drone.global_init_pose)
    
    self.geo_helper = Geo_helper()
    init_lat, init_lon = self.initial_gps_pose.latitude, self.initial_gps_pose.longitude
    
    lst_of_xyz = []
    
    for wp in self.wps:
      wp_lat, wp_lon =wp[0], wp[1]
      x,y,z = self.geo_helper.lla_to_enu(
        lla= [wp_lat,wp_lon, 0], 
        origin_lla=[init_lat, init_lon, 0]
        )
      lst_of_xyz.append([x,y,z])
    # self.write_list(directory+"xyz_coor.txt", lst_of_xyz)
    # self.write_list(directory+"lla_coor.txt", self.wps) 
    return lst_of_xyz


  def write_list(self, save_loc, a_list):
    with open(save_loc, 'w') as fp:
      json.dump(a_list, fp)
      print('Done writing list into a binary file')

def read_list(save_loc):
  with open(save_loc,'rb') as fp:
    n_list = json.load(fp)
    return n_list
      
        
if __name__ == '__main__':
  rclpy.init()
  t = Palm_tree()
  directory = "/home/ds/Documents/gh/ardugzros2/src/tutorial_2/"
  # coor_dir = directory+"geoids/xyz_coor.txt"
  model_loc = directory+"models/palm_tree_young_mature/model.sdf"
  
  stdscr = curses.initscr()
  curses.cbreak()
  curses.curs_set(1) # hide cursor
  stdscr.nodelay(True) # make getch() non-blocking
  
  # stdscr.keypad(True)
  
  
  i=0
  while True:
    key = stdscr.getch()
    if key != -1:
      stdscr.addstr(f"Key pressed: {key} ({chr(key)})\n")
      stdscr.refresh()
      if key == ord('q'):
        print("You pressed a key!")
        rclpy.spin_once(t.drone)
        lst = t.get_coords()      
        for xyz in lst:
          x, y, z = xyz
          print(x,y)
          spawn_models = ["ros2", "run", "ros_gz_sim","create","-file",f"{model_loc}","-name",f"tree_{i}","-x",f"{y}","-y",f"{-x}"]
          subprocess.run(spawn_models) 
          i+=1
      if key == ord('e'):
        break
  
    # spawn_models = f"gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: '{model_loc}', name: 'tree_{i}''"
    # spawn_models= ["gz","service","-s", "/world/empty/create", "--reqtype", "gz.msgs.EntityFactory","--reptype", "gz.msgs.Boolean","--timeout","1000","--req", "'sdf_filename:", f'"{model_loc}",', "name:",f'"tree_{i}"', "","'"]
    # subprocess.run(spawn_models) 
    # "ros2 run ros_gz_sim create -file my_model.sdf -name new_name_for_model -x 0.1 -y 0.0 -z 2.0"
    