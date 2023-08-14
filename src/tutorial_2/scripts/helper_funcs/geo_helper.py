#!/usr/bin/env python3 
# Path funcs
import os
from ament_index_python.packages import get_package_share_directory

# Math funcs
import numpy as np
import scipy.spatial.transform   

# Geo funcs
""" TEMPORARY removed until implemented without external lib"""
from pyproj import Transformer 
from pygeodesy.geoids import GeoidPGM


class Geo_helper():
  def __init__(self):
    cur_dir = get_package_share_directory("tutorial_2")
    geoid_file_dir = os.path.join(cur_dir, "geoids","egm96-5.pgm")
    self._egm96 = GeoidPGM(geoid_file_dir)
    
  def amsl_of_latlong(self, lat, lon):
    """
    Gets AMSL (Above Mean Sea Level) Altitude
    From Latitude and Longitude using geoids or gridmap
    
    Parameters:
    -----------
    lat: Latitude  m
    lon: Longitude m
    
    Returns
    alt: Altitude(AMSL) m
    """
    return self._egm96.height(lat,lon)
  
  def lla_to_ecef(self, lat, lon, alt):
    """
    Convert LLA (Latitude, Longitude, Altitude) coordinates 
    to ECEF (Earth-Centered Earth-Fixed) coordinates
    
    Parameters:
    -----------
    Lat: Latitude  m
    lon: Longitude m
    alt: Altitude  m
    
    Returns:
    --------
    xyz: an array of Earth Centered Earth Fixed
    """
    # WGS84 ellipsoid constants
    a = 6378137.0  # semi-major axis [m]
    b = 6356752.314245  # semi-minor axis [m]
    f = (a - b) / a  # flattening
    e_sq = f * (2 - f)  # eccentricity squared

    # convert degrees to radians
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # N, the radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e_sq * np.sin(lat_rad)**2)

    # calculate ECEF coordinates
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = ((1 - e_sq) * N + alt) * np.sin(lat_rad)

    return np.array([x, y, z])
  

  def lla_to_enu(self, lla, origin_lla):
    """
    Convert ECEF coordinates to ENU coordinates using a reference point.
    
    LLA to ENU = LLA -> ECEF -> ENU(xyz)
    Parameters:
    -----------
    ecef: numpy array of shape (3,)
        The ECEF coordinates to convert.
    origin_ecef: numpy array of shape (3,)
        The ECEF coordinates of the reference point.

    Returns:
    --------
    enu: numpy array of shape (3,)
        The ENU coordinates of the input point relative to the reference point.
    """
    lat, lon, alt = lla[0],lla[1],lla[2]
    lat_org, lon_org, alt_org = origin_lla[0],origin_lla[1], origin_lla[2]
    x,y,z = self.lla_to_ecef(lat, lon, alt)
    x_org, y_org, z_org = self.lla_to_ecef(lat_org, lon_org, alt_org)
    
    # Convert the reference point to ECEF coordinates
    vec=np.array([[ x-x_org, y-y_org, z-z_org]]).T

    rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
    rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

    rotMatrix = rot1.dot(rot3)    
  
    enu = rotMatrix.dot(vec).T.ravel()
    return enu.T
  
  def enu_to_lla(self, x,y,z, lat_org, lon_org, alt_org):
      """
      Convert ENU or XYZ to Lat Lon ALt, but am too lazy to impl
      SOOOOO I just got it from pyproj to write it out.
      """
      transformer1 = Transformer.from_crs(
          {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
          {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
          )
      transformer2 = Transformer.from_crs(
          {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
          {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
          )
      
      x_org, y_org, z_org = transformer1.transform( lon_org,lat_org,  alt_org,radians=False)
      ecef_org=np.array([[x_org,y_org,z_org]]).T
      
      rot1 =  scipy.spatial.transform.Rotation.from_euler('x', -(90-lat_org), degrees=True).as_matrix()#angle*-1 : left handed *-1
      rot3 =  scipy.spatial.transform.Rotation.from_euler('z', -(90+lon_org), degrees=True).as_matrix()#angle*-1 : left handed *-1

      rotMatrix = rot1.dot(rot3)

      ecefDelta = rotMatrix.T.dot( np.array([[x,y,z]]).T )
      ecef = ecefDelta+ecef_org
      lon, lat, alt = transformer2.transform( ecef[0,0],ecef[1,0],ecef[2,0],radians=False)

      return [lat,lon,alt]

  # def enu_to_lla2(self, x,y,z, lat_org,lon_org, alt_org):
    

if __name__ == '__main__':
  # The local coordinate origin (Zermatt, Switzerland)
  lat_org = 46.017 # deg
  lon_org = 7.750  # deg
  alt_org = 1673   # meters

  # The point of interest
  lat = 45.976  # deg
  lon = 7.658   # deg
  alt = 4531    # meters

  geo_helper = Geo_helper()
  
  ref_lat, ref_lon, ref_alt =37.7749, -122.4194, 10.0
  res1 = geo_helper.lla_to_enu([ref_lat, ref_lon, ref_alt],[ref_lat, ref_lon, ref_alt])
  
  print(res1)
  x,y,z = -20,-20,1
  res2 = enu_to_lla(x,y,z, lat_org, lon_org, alt_org)
  print (res2)

  
  # print(geo_helper.amsl_of_latlong(lat_org,lon_org))
    