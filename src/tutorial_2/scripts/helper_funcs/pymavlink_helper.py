import time
from pymavlink import mavutil
from pymavlink.mavutil import mavlink
import logging
import math
from MAVProxy.modules.lib import mp_util
logging.basicConfig(level=logging.DEBUG)

# Create a class
class Mav_helper():
  def __init__(self):
    # Create the connection to the top-side computer as companion computer/autopilot
    self.master = mavutil.mavlink_connection('udpin:localhost:14550', source_system=1)
    self.master.wait_heartbeat()
    self.logger = logging.getLogger(self.__class__.__name__)
    self.request_message_interval(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 60)

  def request_message_interval(self, message_id: int, frequency_hz: float):
    self.master.mav.command_long_send(
      self.master.target_system, self.master.target_component,
      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
      message_id, # The MAVLink message ID
      1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
      0, 0, 0, 0, # Unused parameters
      0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )
    
  def arm(self):
    self.master.arducopter_arm()
    print("Waiting for the vehicle to arm")
    self.master.motors_armed_wait()
    print('Armed!')
    
  def disarm(self):
    self.master.arducopter_disarm() 
    print("Waiting for the vehicle to disarm")
    self.master.motors_disarmed_wait()
    print('DisArmed!')
  
  def set_mode(self,mode):
    try:
      if mode not in self.master.mode_mapping():
        raise AssertionError
      self.logger.info(f"Trying to use Mode: [{mode}]")
    except AssertionError as e:
      self.logger.error(f"Invalid Mode [{mode}]\nTry: {list(self.master.mode_mapping().keys())}")
      msg = self.master.recv_match(type = 'HEARTBEAT', blocking = True)
      mode = mavutil.mode_string_v10(msg)
      self.logger.error(f"Defaulted to Current Mode: [{mode}]")
    finally:
      mode_id = self.master.mode_mapping()[mode]
      self.master.set_mode(mode_id)

      while True:
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg:
          ack_msg = ack_msg.to_dict()
          if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
          self.logger.info(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
          break
  
  def takeoff(self,desired_alt):
    self.logger.info(f"Taking off to {desired_alt}m")
    alt = self.master.location(relative_alt=True).alt
    
    self.master.mav.command_long_send(
      self.master.target_system,
      self.master.target_component,
      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
      0, 0, 0, 0, 0, 0, 0, desired_alt)
    
    while True:
      ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
      if ack_msg is None:
        continue
      if ack_msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        self.logger.info(mavutil.mavlink.enums['MAV_RESULT'][ack_msg.result].description)
        break
    while alt <= desired_alt*0.95:
      alt = self.master.location(relative_alt=True).alt
      # altmsg = self.master.recv_match(type='VFR_HUD', blocking=False)
      # if altmsg is None:
      #   continue
      # if (altmsg.alt-current_alt) >= alt*0.95:
      #   self.logger.info(f"Reached Target Altitude : [{altmsg.alt}m]")
      #   break
    self.logger.info(f"Reached Takeoff Altitude : [{alt}m]")
  
  def land(self):
    self.logger.info("Landing")
    alt = self.master.location(relative_alt=True).alt
    self.master.mav.command_long_send(
      self.master.target_system,
      self.master.target_component,
      mavutil.mavlink.MAV_CMD_NAV_LAND,
      0, 0, 0, 0, 0, 0, 0, 0)
    
    while True:
      ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
      if ack_msg is None:
        continue
      if ack_msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
        self.logger.info(mavutil.mavlink.enums['MAV_RESULT'][ack_msg.result].description)
        break
    while alt >= 0.05:
      alt = self.master.location(relative_alt=True).alt
      # throttle = self.master.recv_match(type='VFR_HUD', blocking=True).throttle
      # if throttle is None:
      #   continue
      # # print(throttle)
      # if throttle <= 10:
      #   self.logger.info(f"Landed at Altitude : [{current_alt}m]")
      #   break
    self.logger.info(f"Landed at Altitude : [{alt}m]")
    self.disarm()
  
  def do_RTL(self, timeout=250):
    """Return, land"""
    self.set_mode("RTL")
    # self.master.set_rc(3,1500)
    tstart = self.get_sim_time()
    while self.get_sim_time_cached() < tstart + timeout:
      msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
      alt = msg.relative_alt / 1000.0 # mm -> m
      home_distance = self.distance_to_home(use_cached_home=True)
      home = ""
      if alt <= 1 and home_distance < 10:
        home = "HOME"
      # our post-condition is that we are disarmed:
      if not self.master.motors_armed():
        if home == "":
          raise Exception("Did not get to home")
        # success!
        return
    raise TimeoutError("Did not get home and disarm")
  
  def distance_to_home(self, use_cached_home=False):
    m = self.master.messages.get("HOME_POSITION", None)
    if use_cached_home is False or m is None:
      m = self.poll_home_position()
    here = self.master.recv_match(type='GLOBAL_POSITION_INT',
                                blocking=True)
    return self.get_distance_int(m, here)
  
  def poll_home_position(self, quiet=True, timeout=30):
    old = self.master.messages.get("HOME_POSITION", None)
    tstart = self.get_sim_time()
    while True:
      if self.get_sim_time_cached() - tstart > timeout:
        raise TimeoutError("Failed to poll home position")
      try:
        self.master.run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,0,0,0,0,0,0,0,quiet=quiet)
      except ValueError as e:
        continue
      m = self.master.messages.get("HOME_POSITION", None)
      if m is None:
        continue
      if old is None:
        break
      if m._timestamp != old._timestamp:
        break
    return m
  
  @staticmethod
  def get_distance_accurate(loc1, loc2):
      """Get ground distance between two locations."""
      try:
          lon1 = loc1.lng
          lon2 = loc2.lng
      except AttributeError:
          lon1 = loc1.lon
          lon2 = loc2.lon

      return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)

  @staticmethod
  def get_latlon_attr(loc, attrs):
      '''return any found latitude attribute from loc'''
      ret = None
      for attr in attrs:
          if hasattr(loc, attr):
              ret = getattr(loc, attr)
              break
      if ret is None:
          raise ValueError("None of %s in loc" % str(attrs))
      return ret

  @staticmethod
  def get_lat_attr(loc):
      '''return any found latitude attribute from loc'''
      return Mav_helper.get_latlon_attr(loc, ["lat", "latitude"])

  @staticmethod
  def get_lon_attr(loc):
      '''return any found latitude attribute from loc'''
      return Mav_helper.get_latlon_attr(loc, ["lng", "lon", "longitude"])

  @staticmethod
  def get_distance_int(loc1, loc2):
      """Get ground distance between two locations in the normal "int" form
      - lat/lon multiplied by 1e7"""
      loc1_lat = Mav_helper.get_lat_attr(loc1)
      loc2_lat = Mav_helper.get_lat_attr(loc2)
      loc1_lon = Mav_helper.get_lon_attr(loc1)
      loc2_lon = Mav_helper.get_lon_attr(loc2)

      return Mav_helper.get_distance_accurate(
          mavutil.location(loc1_lat*1e-7, loc1_lon*1e-7),
          mavutil.location(loc2_lat*1e-7, loc2_lon*1e-7))

      dlat = loc2_lat - loc1_lat
      dlong = loc2_lon - loc1_lon

      dlat /= 10000000.0
      dlong /= 10000000.0

  
  def fly_guided_move_local(self, x, y, z_up, timeout=100):
    """move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED"""
    startpos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

    tstart = self.get_sim_time()
    while True:
      if self.get_sim_time_cached() - tstart > timeout:
          raise Exception("Did not start to move")
      # send a position-control command
      self.master.mav.set_position_target_local_ned_send(
          0, # timestamp
          self.master.target_system,
          self.master.target_component,
          mavutil.mavlink.MAV_FRAME_LOCAL_NED,
          0b1111111111111000, # mask specifying use-only-x-y-z
          x, # x
          y, # y
          -z_up,# z
          0, # vx
          0, # vy
          0, # vz
          0, # afx
          0, # afy
          0, # afz
          0, # yaw
          0, # yawrate
      )
      m = self.master.recv_match(type='VFR_HUD', blocking=True)
      print("%s" % m)
      if m.groundspeed > 0.5:
        break

    self.logger.info("Waiting for vehicle to stop...")
    self.wait_groundspeed(1, 100, timeout=timeout)

    stoppos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    self.logger.info("stop_pos=%s" % str(stoppos))

    x_achieved = stoppos.x - startpos.x
    if x_achieved - x > 1:
        raise Exception("Did not achieve x position: want=%f got=%f" % (x, x_achieved))

    y_achieved = stoppos.y - startpos.y
    if y_achieved - y > 1:
        raise Exception("Did not achieve y position: want=%f got=%f" % (y, y_achieved))

    z_achieved = stoppos.z - startpos.z
    if z_achieved - z_up > 1:
        raise Exception("Did not achieve z position: want=%f got=%f" % (z_up, z_achieved))
  
  def fly_guided_move_global_relative_alt(self, lat, lon, alt):
      startpos = self.master.recv_match(type='GLOBAL_POSITION_INT',
                                      blocking=True)

      tstart = self.get_sim_time()
      while True:
          if self.get_sim_time_cached() - tstart > 200:
              raise Exception("Did not move far enough")
          # send a position-control command
          self.master.mav.set_position_target_global_int_send(
              0, # timestamp
              1, # target system_id
              1, # target component id
              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
              0b1111111111111000, # mask specifying use-only-lat-lon-alt
              lat, # lat
              lon, # lon
              alt, # alt
              0, # vx
              0, # vy
              0, # vz
              0, # afx
              0, # afy
              0, # afz
              0, # yaw
              0, # yawrate
          )
          pos = self.master.recv_match(type='GLOBAL_POSITION_INT',
                                    blocking=True)
          delta = self.get_distance_int(startpos, pos)
          self.logger.info("delta=%f (want >10)" % delta)
          if delta > 10:
              break
  
  
  def wait_groundspeed(self, gs_min, gs_max, timeout=30):
    """Wait for a given ground speed range."""
    self.logger.info("Waiting for groundspeed between %.1f and %.1f" %
                  (gs_min, gs_max))
    last_print = 0
    tstart = self.get_sim_time()
    while self.get_sim_time_cached() < tstart + timeout:
      m = self.master.recv_match(type='VFR_HUD', blocking=True)
      if self.get_sim_time_cached() - last_print > 1:
          self.logger.info("Wait groundspeed %.3f, target:%.3f" %
                        (m.groundspeed, gs_min))
          last_print = self.get_sim_time_cached()
      if m.groundspeed >= gs_min and m.groundspeed <= gs_max:
          return True
    raise Exception("Failed to attain groundspeed range")
  
  def get_sim_time(self):
      """Get SITL time in seconds."""
      m = self.master.recv_match(type='SYSTEM_TIME', blocking=True)
      return m.time_boot_ms * 1.0e-3

  def get_sim_time_cached(self):
      """Get SITL time in seconds."""
      x = self.master.messages.get("SYSTEM_TIME", None)
      if x is None:
          raise Exception("No cached time available")
      return x.time_boot_ms * 1.0e-3

if __name__=="__main__":
  obj = Mav_helper()
  obj.set_mode("GUIDED")
  obj.arm()
  obj.takeoff(1)
  obj.fly_guided_move_local(40,40,6)
  obj.do_RTL()
  obj.land()