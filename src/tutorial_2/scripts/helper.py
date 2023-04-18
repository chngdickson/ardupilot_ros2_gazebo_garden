#!/usr/bin/env python3 
# Python funcs
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from transforms3d.euler import euler2quat, quat2euler
import threading
import asyncio

# ROS2 essentials
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import message_filters

# Message types
from mavros_msgs.msg import State, GlobalPositionTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

# Service types
from mavros_msgs.srv import CommandBool,CommandTOL
from functools import partial

class Ardu_Ros_Connect(Node):
  def __init__(self):
    super().__init__(self.__class__.__name__)
    timer_speed = 0.00000001
    # Pub sub Messages
    self.curr_state = State()
    self.curr_pose = Odometry()
    self.waypoint = PoseStamped()
    self.curr_global_pose = NavSatFix()
    
    # Class Internals
    self.local_init_pose = Point()
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
    
    # Publishers
    self.pub_local_pos = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
    self.pub_global_pos = self.create_publisher(GeoPoseStamped,"/mavros/setpoint_position/global", 10)
    
    # Clients
    self.arming_client = self.create_client(CommandBool,"/mavros/cmd/arming")
    self.takeoff_client = self.create_client(CommandTOL,"/mavros/cmd/takeoff")
    self.land_client = self.create_client(CommandTOL,"/mavros/cmd/land")
  
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
    self.get_logger().info("wait2start")
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
    result = False
    async def call_client():
      nonlocal result
      try:
        future = self.arming_client.call_async(arm_request)
        result = await future
      except Exception:
        pass
    
    timer = self.create_timer(1, call_client, callback_group=ReentrantCallbackGroup())
    
    while not self.curr_state.armed or not result.success:
      rclpy.spin_once(self)
    else:
      timer.cancel()
      self.get_logger().info(f"{arm_mode} completed!  [{self.curr_state.armed}] [{result.success}]")
      return True
  
  def initialize_local_frame(self):
    for i in range(30):
      rclpy.spin_once(self, timeout_sec=0.1)
      w,x,y,z,curr_heading = self._get_curr_euler(self.curr_pose)
      self.local_init_heading += degrees(curr_heading)
      self.local_init_pose.x += x
      self.local_init_pose.y += y
      self.local_init_pose.z += z
    
    self.local_init_heading  /= 30.0
    self.local_init_pose.x /= 30.0
    self.local_init_pose.y /= 30.0
    self.local_init_pose.z /= 30.0
    self.get_logger().info("Coordinate offset set" )
    self.get_logger().info(f"The X-Axis is facing: {self.local_init_heading}")

  def takeoff(self, takeoff_alt: float):
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
    
    self.set_destination(0,0,takeoff_alt,0, publish=False)
    
    if future.result().success:
      self.get_logger().info(f"Takeoff Initiated")
      while not self.check_waypoint_reached():
        rclpy.spin_once(self, timeout_sec=0.01)
      self.get_logger().info(f"Takeoff Completed! [{self.check_waypoint_reached()}] ")
      return None
    else:
      self.get_logger().error(f"Takeoff FAILED")
      raise Exception()
  
  """
  END PRE-FLIGHT INITIALIZATION
  """
  def returnToHome(self):
    # land_res = CommandTOL.Response()
    land_req = CommandTOL.Request()
    land_req.altitude = 0.0
    land_req.latitude = 0.0
    land_req.longitude = 0.0
    land_req.min_pitch = 0.0
    land_req.yaw = 0.0
    
    
    while not self.land_client.wait_for_service(1.0):
      self.get_logger().info("land service not available, waiting again...")
      return self.returnToHome()
    
    self.get_logger().info("Sending land signal")
    future = self.land_client.call_async(land_req)
    rclpy.spin_until_future_complete(self, future)
    
    if future.result().success:
      self.get_logger().info(f"Landing Initiated")
      while not str(self.curr_state.mode) == "LAND":
        rclpy.spin_once(self, timeout_sec=0.01)
      self.get_logger().info(f"Landing Completed! [{self.curr_state.mode}]")
      return None
    else:
      self.get_logger().error(f"Takeoff Failed")
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
    
  def set_destination(self, x, y, z, heading, publish:bool=True):    
    self.waypoint.pose.position = self.set_pos(x,y,z)
    self.waypoint.pose.orientation = self.set_heading(heading)
    if publish:
      self.pub_local_pos.publish(self.waypoint)
    return None
  
  def set_destination_global_lla(self,lat: float, long: float, alt: float, heading: float):
    lat_long_alt_msg = GeoPoseStamped()
    lat_long_alt_msg.pose.position.latitude  = lat
    lat_long_alt_msg.pose.position.longitude = long
    lat_long_alt_msg.pose.position.altitude = alt
    lat_long_alt_msg.pose.orientation = self.set_heading(heading)
    self.pub_global_pos.publish(lat_long_alt_msg)
    return
  
  def check_waypoint_reached(self, pos_tol=0.2, head_tol=0.2)-> bool:    
    # Pos Diff based on Distance calc
    dx = abs(self.waypoint.pose.position.x - self.curr_pose.pose.pose.position.x)
    dy = abs(self.waypoint.pose.position.y - self.curr_pose.pose.pose.position.y)
    dz = abs(self.waypoint.pose.position.z - self.curr_pose.pose.pose.position.z)
    dpos = sqrt(pow(dx,2) + pow(dy,2) +pow(dz,2))
    
    # Heading diff based on angle diff
    cosErr = cos(radians(self.curr_heading)) - cos(radians(self.local_desired_heading))
    sinErr = sin(radians(self.curr_heading)) - sin(radians(self.local_desired_heading))
    dHeading = sqrt(pow(cosErr,2) + pow(sinErr,2))
    
    if dpos < pos_tol and dHeading < head_tol:
      return True
    else:
      return False
  

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
  executor = MultiThreadedExecutor(num_threads=4)
  thread = threading.Thread(target=run_executor, args=(executor, drone), daemon=True)
  thread.start()
  
  drone.wait4connect()
  
  drone.wait4start()
  
  drone.arm_disarm(arming=True)
  
  drone.initialize_local_frame()
  
  drone.takeoff(2.0)

  goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
          [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
  i = 0
  while i < len(goals):
    drone.set_destination(
      x=goals[i][0], y=goals[i][1], z=goals[i][2], heading=goals[i][3]
    )
    # print("HELLO AM I WORKING")
    rclpy.spin_once(drone, timeout_sec=0.0001)
    if drone.check_waypoint_reached():
      i += 1
      
  drone.returnToHome()
  
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
  