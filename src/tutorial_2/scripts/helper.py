#!/usr/bin/env python3 
# My personal helpers
from ros_helper import *
from math_helper import *
from geo_helper import *

# Python funcs
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from transforms3d.euler import euler2quat, quat2euler
import threading
import asyncio
import time
import numpy as np

# ROS2 essentials
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client, SrvTypeRequest
import message_filters

# Message types
from mavros_msgs.msg import State, GlobalPositionTarget, WaypointList, Waypoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

# Service types
from mavros_msgs.srv import CommandBool,CommandTOL, WaypointClear, WaypointPull, SetMode


class Ardu_Ros_Connect(Node):
  def __init__(self):
    super().__init__(self.__class__.__name__)
    # This substitude NODE is IMPORTANT for running spins within a subscriber.
    self.sub_node = rclpy.create_node('sub_node')
    timer_speed = 0.00000001
    self._wpoints = []
    # Helper classes
    self.geo = Geo_helper()
    
    # Pub sub Messages
    self.curr_state = State()
    self.curr_pose = Odometry()
    self.waypoint = PoseStamped()
    self.curr_global_pose = NavSatFix()
    
    # Class Internals
    self.local_init_pose = Point()
    self.global_init_pose = NavSatFix()
    self.local_init_heading = 0.0
    self.local_desired_heading = 0.0
    self.curr_heading = 0.0
    
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
    # Subscribers
    self.sub_state = self.create_subscription(State, '/mavros/state', self.cb_state, 10)
    self.sub_local_pose = self.create_subscription(Odometry,'/mavros/global_position/local', self.cb_local_pose, qos_profile)
    self.sub_global_pose = self.create_subscription(NavSatFix,'/mavros/global_position/global', self.cb_global_pose, qos_profile)
    self.sub_waypoints = self.create_subscription(WaypointList,'/mavros/mission/waypoints',self.cb_waypoints, 10)
    
    # Publishers
    self.pub_state = self.create_publisher(State, "/mavros/state", 10)
    self.pub_local_pos = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
    self.pub_global_pos = self.create_publisher(GeoPoseStamped,"/mavros/setpoint_position/global", 10)
    
    # Clients
    self.arming_client = self.create_client(CommandBool,"/mavros/cmd/arming")
    self.takeoff_client = self.create_client(CommandTOL,"/mavros/cmd/takeoff")
    self.land_client = self.create_client(CommandTOL,"/mavros/cmd/land")
    self.waypoint_pull_client = self.create_client(WaypointPull,"/mavros/mission/pull")
    self.waypoint_clr_client = self.create_client(WaypointClear,"/mavros/mission/clear")
    self._waypoint_clr_client = self.sub_node.create_client(WaypointClear,"/mavros/mission/clear")
    self.state_client = self.create_client(SetMode, "/mavros/set_mode")
    self._state_client = self.sub_node.create_client(SetMode,"/mavros/set_mode")
    
  """
  PRIVATE FUNCTIONS
  """
  def _get_curr_euler(self, curr_pose:Odometry):
    w, x, y, z = (
      curr_pose.pose.pose.orientation.w,
      curr_pose.pose.pose.orientation.x,
      curr_pose.pose.pose.orientation.y,
      curr_pose.pose.pose.orientation.z,
    )
    
    curr_heading = atan2((2 * (w * z + x * y)),(1 - 2 * (pow(y, 2) + pow(z, 2))))
    return w,x,y,z,curr_heading
  """
  END PRIVATE FUNCTIONS
  """
  
  """
  START PRE-FLIGHT INITIALIZATION
  """
  def cb_state(self, curr_state):
    self.curr_state = curr_state 
  
  def cb_local_pose(self, odom_msg:Odometry):
    self.curr_pose = odom_msg
    w,x,y,z,curr_heading = self._get_curr_euler(self.curr_pose)
    self.curr_heading = degrees(curr_heading) - self.local_init_heading
  
  def cb_global_pose(self, msg:NavSatFix):
    self.curr_global_pose = msg
    self.curr_global_pose.altitude = abs(msg.altitude)
    
  def wait4connect(self):
    self.get_logger().info("wait4connect")
    while rclpy.ok() and not self.curr_state.connected:
      rclpy.spin_once(self, timeout_sec=0.0001)
    else:
      if self.curr_state.connected:
        self.get_logger().info("Connected to FCU")
        return True
      else:
        self.get_logger().error("error connecting to drone") 
        return -1
  
  def wait4start(self):
    self.get_logger().info("wait4start")
    while rclpy.ok() and self.curr_state.mode != "GUIDED":
      rclpy.spin_once(self, timeout_sec=0.001)
    else:
      if self.curr_state.mode == "GUIDED":
        self.get_logger().info("Mode set to Guided, starting Mission")
        return True
      else:
        self.get_logger().error("error starting mission") 
        return -1

  def arm_disarm(self, arming=True):
    arm_request = CommandBool.Request()
    arm_request.value = arming
    arm_mode = "Arming" if arming else "Disarming"
    
    while not self.arming_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info(f"{arm_mode} service not available, waiting again...")
      return self.arm_disarm(arming=arming)
    
    self.get_logger().info(f"sending {arm_mode} Signal..." )
    async def call_client():
      try:
        future = self.arming_client.call_async(arm_request)
        result = await future
      except Exception:
        pass
    
    timer = self.create_timer(1, call_client, callback_group=ReentrantCallbackGroup())
    
    while not self.curr_state.armed:
      rclpy.spin_once(self)
    else:
      timer.cancel()
      self.get_logger().info(f"{arm_mode} completed!  [{self.curr_state.armed}]")
      return True
  
  def initialize_local_and_global_frame(self):    
    # Class Internals
    self.local_init_pose = Point()
    self.global_init_pose = NavSatFix()
    self.local_init_heading = 0.0
    
    for i in range(30):
      rclpy.spin_once(self, timeout_sec=0.1)
      w,x,y,z,curr_heading = self._get_curr_euler(self.curr_pose)
      self.local_init_heading += degrees(curr_heading)
      self.local_init_pose.x += x
      self.local_init_pose.y += y
      self.local_init_pose.z += z
      
      cgp = self.curr_global_pose
      lat, lon, alt= cgp.latitude, cgp.longitude, cgp.altitude
      self.global_init_pose.latitude += lat
      self.global_init_pose.longitude += lon
      self.global_init_pose.altitude += alt
      time.sleep(0.05)
    self.local_init_heading  /= 30.0
    self.local_init_pose.x /= 30.0
    self.local_init_pose.y /= 30.0
    self.local_init_pose.z /= 30.0
    
    self.global_init_pose.latitude /= 30.0
    self.global_init_pose.longitude /= 30.0
    self.global_init_pose.altitude /= 30.0
    self.global_init_pose.altitude = abs(self.geo.amsl_of_latlong(self.global_init_pose.latitude, self.global_init_pose.longitude)) - abs(self.global_init_pose.altitude)
    self.get_logger().info("Coordinate offset set" )
    self.get_logger().info(f"The X-Axis is facing: {self.local_init_heading}")

  def takeoff(self, takeoff_alt: float, n=0):
    self.initialize_local_and_global_frame()
    takeoff_req = CommandTOL.Request()
    takeoff_req.altitude = takeoff_alt
    takeoff_req.latitude=0.0
    takeoff_req.longitude=0.0
    takeoff_req.min_pitch=0.0
    takeoff_req.yaw = 0.0
    
    
    while not self.takeoff_client.wait_for_service(1.0):
      self.get_logger().info("takeoff service not available, waiting again...")
      return self.takeoff(takeoff_alt)
    
    self.get_logger().info("Sending Takeoff signal")
    future = self.takeoff_client.call_async(takeoff_req)
    rclpy.spin_until_future_complete(self, future)
    
    self.set_destination_global_lla(
      self.global_init_pose.latitude,
      self.global_init_pose.longitude,
      self.global_init_pose.altitude+takeoff_alt,
      self.local_init_heading, 
      publish=False
      )
    # self.set_destination_euler(self.local_init_pose.x,self.local_init_pose.y,takeoff_alt,0, publish=False)
    
    if future.result() is not None:
      result : CommandTOL.Response = future.result()
      if result.success:
        self.get_logger().info(f"Takeoff Initiated")
        while not self.check_waypoint_reached_global(pos_tol=0.3, head_tol=180):
          rclpy.spin_once(self, timeout_sec=0.001)
        time.sleep(0.5)
        self.get_logger().info(f"Takeoff Completed! ")
        return None
      else:
        self.get_logger().warn(f"Takeoff Failed! It's Probably already Taken off, We'll try 2 more times ")
        time.sleep(5.0)
        self.arm_disarm(True)
        return self.takeoff(takeoff_alt, n+1) if n<3 else None
    else:
      return self.takeoff(takeoff_alt)
  
  """
  END PRE-FLIGHT INITIALIZATION
  """
  def returnToHome(self):
    self.get_logger().info("GOING HOME..")
    self.go_destination_global_lla(
      self.global_init_pose.latitude,
      self.global_init_pose.longitude,
      self.curr_global_pose.altitude,
      self.local_init_heading
      )
    
    # land_res = CommandTOL.Response()
    land_req = CommandTOL.Request()
    land_req.altitude = 0.0
    land_req.latitude = 0.0
    land_req.longitude = 0.0
    land_req.min_pitch = 0.0
    land_req.yaw = self.local_init_heading
    
    
    while not self.land_client.wait_for_service(1.0):
      self.get_logger().info("land service not available, waiting again...")
      return self.returnToHome()
    
    self.get_logger().info("Sending land signal")
    future = self.land_client.call_async(land_req)
    rclpy.spin_until_future_complete(self, future)
    
    self.set_destination_global_lla(
      self.global_init_pose.latitude,
      self.global_init_pose.longitude,
      self.global_init_pose.altitude,
      self.local_init_heading, 
      publish=False
      )
    if future.result().success:
      self.get_logger().info(f"Landing Initiated")
      while not str(self.curr_state.mode) == "LAND" or not self.check_waypoint_reached_global(0.2):
        rclpy.spin_once(self, timeout_sec=0.01)
      time.sleep(3.0) # Sleep for 3 seconds to wait prop inertia
      self.get_logger().info(f"Landing Completed! [{self.curr_state.mode}]")
      return None
    else:
      self.get_logger().error(f"Landing Failed")
      raise Exception()



  
  def set_heading(self, heading)->Quaternion:
    self.local_desired_heading = heading
    heading += self.local_init_heading
    
    yaw, pitch, roll = radians(heading), 0.0, 0.0
    qx,qy,qz,qw = euler2quat(yaw,pitch,roll)
    q = Quaternion()
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return q
    
  def set_pos(self,x,y,z):
    theta = radians((self.local_init_heading-90))
    
    Xlocal = x*cos(theta) - y*sin(theta)
    Ylocal = x*sin(theta) + y*cos(theta)
    Zlocal = z
    
    x = Xlocal + self.local_init_pose.x
    y = Ylocal + self.local_init_pose.y
    z = Zlocal + self.local_init_pose.z
    
    p = Point()
    p.x, p.y, p.z = x, y, z
    return p
  
  def go_destination_euler(self, x, y, z, heading, publish:bool=True):
    self.set_destination_euler(x, y, z, heading, True)
    while not self.check_waypoint_reached(0.3):
      rclpy.spin_once(self, timeout_sec=0.001)
      self.set_destination_euler(x, y, z, heading, True)
    return None
  
  def go_destination_global_lla(self,lat: float, long: float, alt: float, heading: float):
    self.set_destination_global_lla(lat, long, alt ,heading)
    while not self.check_waypoint_reached_global(0.3):
      rclpy.spin_once(self, timeout_sec=0.001)
      self.set_destination_global_lla(lat, long, alt ,heading)
    return None
  
  def set_destination_euler(self, x, y, z, heading, publish:bool=True):    
    self.waypoint.pose.position = self.set_pos(x,y,z)
    self.waypoint.pose.orientation = self.set_heading(heading)
    if publish:
      self.pub_local_pos.publish(self.waypoint)
    return None
  
  def set_destination_quat(self, wp:Pose, publish:bool=True):
    self.waypoint.pose.position = wp.position
    self.waypoint.pose.orientation = wp.orientation
    if publish:
      self.pub_local_pos.publish(self.waypoint)
      while not self.check_waypoint_reached():
        rclpy.spin_once(self, timeout_sec=0.0001)
    return None
  
  def set_destination_global_lla(self,lat: float, long: float, alt: float, heading: float, publish:bool=True):
    lat_long_alt_msg = GeoPoseStamped()
    lat_long_alt_msg.pose.position.latitude  = lat
    lat_long_alt_msg.pose.position.longitude = long
    lat_long_alt_msg.pose.position.altitude = alt
    lat_long_alt_msg.pose.orientation = self.set_heading(heading)
    self.lat_long_alt_msg = lat_long_alt_msg
    # print("Publishing",lat,long,alt)
    if publish:
      self.pub_global_pos.publish(lat_long_alt_msg)
    return
  
  def check_waypoint_reached(self, pos_tol=0.2, head_tol=0.5)-> bool:    
    # Pos Diff based on Distance calc
    dx = abs(self.waypoint.pose.position.x - self.curr_pose.pose.pose.position.x)
    dy = abs(self.waypoint.pose.position.y - self.curr_pose.pose.pose.position.y)
    dz = abs(self.waypoint.pose.position.z - self.curr_pose.pose.pose.position.z)
    dpos = sqrt(dx**2 + dy**2 + dz**2)
    
    
    # Heading diff based on angle diff
    cosErr = cos(radians(self.curr_heading)) - cos(radians(self.local_desired_heading))
    sinErr = sin(radians(self.curr_heading)) - sin(radians(self.local_desired_heading))
    dHeading = sqrt(pow(cosErr,2) + pow(sinErr,2))
    
    if dpos < pos_tol and dHeading<head_tol:
      return True
    else:
      # print(dpos, pos_tol, pow(dx,2), pow(dy,2), pow(dz,2), dHeading)
      return False
    
    
  def check_waypoint_reached_global(self, pos_tol, head_tol=0.5)-> bool:
    """
    This function uses the 
    - Global Position for pose
    - Local Orientation for orientation due to lack of impl for global
    The reason is that the current MAVROS implemetation for y coordinate is wrong, There's a drift.
    """
    ref = self.lat_long_alt_msg
    cgp, ref_pos = self.curr_global_pose, ref.pose.position
    lat, lon, alt_amsl = cgp.latitude, cgp.longitude, cgp.altitude
    alt_terrain = abs(self.geo.amsl_of_latlong(lat,lon)) 
    alt_wsg = abs(alt_amsl - alt_terrain)
    
    dx, dy, dz = self.geo.lla_to_enu(
            origin_lla = [ref_pos.latitude,ref_pos.longitude,ref_pos.altitude],
            lla = [lat,lon,alt_wsg]
            )
    # print(
    #   f"Ori [{alt_wsg:.4f} = amsl{alt_amsl:.4f} - Ter{alt_terrain:.4f}/n\
    #   Sub [{ref_pos.altitude:.4f}] abs[{abs(dz):.4f}]"
    # )
    # Pos Diff based on Distance calc
    dx, dy, dz = abs(dx), abs(dy), abs(dz)
    dpos = sqrt(dx**2 + dy**2 + dz**2)
    
    
    # Heading diff based on angle diff
    cosErr = cos(radians(self.curr_heading)) - cos(radians(self.local_desired_heading))
    sinErr = sin(radians(self.curr_heading)) - sin(radians(self.local_desired_heading))
    dHeading = sqrt(pow(cosErr,2) + pow(sinErr,2))
    
    if dpos < pos_tol and dHeading<head_tol:
      return True
    else:
      # print("GLOBAL",dpos, pos_tol, pow(dx,2), pow(dy,2), pow(dz,2), "\n", dHeading)
      return False

  def cb_waypoints(self, msg:WaypointList):
    lst = []
    wpoints = []
    for i, wp in enumerate(msg.waypoints):
      wp:Waypoint=wp
      if wp.command == 16 and i != 0:
        # pos = Pose()
        gip = self.global_init_pose
        wpoints.append([wp.x_lat, wp.y_long, wp.z_alt,wp.param4])
        # wpoints.append(pos)
    if len(wpoints) > 0:
      self._wpoints = wpoints
  
  def get_waypoints(self)-> list:
    wps = self._wpoints
    if len(wps) == 0:
      assert IndexError()
    else:
      print("All good, ready to fly.\n\n")
      for wp in wps:
        print(wp,'\n')
    return wps
    
      
  def clr_waypoints(self):
    return generic_service_call(
      node = self,
      client = self.waypoint_clr_client,
      client_req=WaypointClear.Request(),
      client_res=WaypointClear.Response(),
      name_of_srv="External_clr_waypoint",
      timeout=3.0
      )
    
  def _clr_waypoints(self):
    # request, response= WaypointClear.Request(), WaypointClear.Response()
    # future = self._waypoint_clr_client.call_async(request)
    # rclpy.spin_until_future_complete(self.sub_node, future)
    
    # if future.result() is not None:
    #   result: WaypointClear.Request = future.result()
    #   response.success = result.success
    #   return response
    # else:
    #   self.get_logger().error("exception while")
    return generic_service_call(
      node = self.sub_node,
      client = self._waypoint_clr_client,
      client_req=WaypointClear.Request(),
      client_res=WaypointClear.Response(),
      name_of_srv="Internal_pull_waypoint",
      timeout=3.0
      )
      
  def pull_waypoints(self):
    return generic_service_call(
      node = self,
      client = self.waypoint_pull_client,
      client_req=WaypointPull.Request(),
      client_res=WaypointPull.Response(),
      name_of_srv="External_pull_waypoint",
      timeout=3.0
      )
  
  def await_waypoints_before_takeoff(self):
    timer, result = generic_service_timer(self, self.waypoint_pull_client, WaypointPull.Request(), 
                                  name_of_srv="External_pull_waypoint",timer_time=1.0)
    self.get_logger().info(f"Please publish Waypoints to Mission Planner! ")
    while len(self._wpoints) == 0:
      rclpy.spin_once(self)
    else:
      timer.cancel()
      self.get_logger().info(f"await_waypoints_before_takeoff completed! There are currently, [{len(self._wpoints)}] wps ")
      self._clr_waypoints()
      return True

  def state_GUIDED(self):
    name_of_srv = "setMode"
    req = SetMode.Request()
    req.base_mode = int(88)
    req.custom_mode = "GUIDED"
    while not self.state_client.wait_for_service(1.0):
      self.get_logger().info(f"{name_of_srv} service not available, waiting again...")
      return self.state_GUIDED()
    
    self.get_logger().info(f"Sending {name_of_srv} signal")
    future = self.state_client.call_async(req)
    rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
    
    if future.result() is not None:
      result : SetMode.Response = future.result()
      # print(result,"\n\n")
      if result.mode_sent:
        self.get_logger().info(f"{name_of_srv} Completed")
      return result
    else:
      self.get_logger().error("exception while")
    
def run_executor(executor, node):
  executor.add_node(node)
  try:
    executor.spin()
  finally:
    node.destroy_node()
    executor.shutdown()
        
def main(args=None):
  rclpy.init(args=args)
  drone = Ardu_Ros_Connect()
  executor = MultiThreadedExecutor(num_threads=2)
  thread = threading.Thread(target=run_executor, args=(executor, drone), daemon=True)
  thread.start()
  drone.wait4connect()
  drone.state_GUIDED()
  drone.wait4start()
  drone.arm_disarm(arming=True) # Pre-checks arm and disarm before checking global_coor
  
  drone.initialize_local_and_global_frame()
  
  drone.await_waypoints_before_takeoff()
  wps = drone.get_waypoints()

  for _ in range(100):
    drone.arm_disarm(arming=True) 
    drone.takeoff(3.0)
    for i, wp in enumerate(wps):
      drone.go_destination_global_lla(
        lat =wp[0],long= wp[1], alt= wp[2], heading = wp[3]
        )
      drone.get_logger().info(f"wp [{i}] reached out of {len(wps)}")
    print("All Waypoints Completed")

    drone.returnToHome()
    drone.state_GUIDED()
    
  """EULERS"""
  # goals = [[0.5619, 0.2834, 3, 0], [5.3218, 0.8631, 3, -90], [5, 5, 3, 0],
  #         [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
  # i = 0
  # for i, wp in enumerate(goals):
  #   drone.set_destination_euler(
  #     x=goals[i][0], y=goals[i][1], z=goals[i][2], heading=goals[i][3]
  #   )
  #   # print("HELLO AM I WORKING")
  #   while not drone.check_waypoint_reached():
  #     rclpy.spin_once(drone, timeout_sec=0.0001)
      
  drone.returnToHome()
  
  rclpy.spin(drone)
  
if __name__ == '__main__':
  main()
  