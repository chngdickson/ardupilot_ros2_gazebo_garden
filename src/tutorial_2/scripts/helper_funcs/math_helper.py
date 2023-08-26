#!/usr/bin/env python3 
import numpy as np
# import quaternion
import math
from math import radians
import transforms3d as tf   
from geometry_msgs.msg import Quaternion, Point

def quaternion_multiply(q1: Quaternion, q2: Quaternion):
    x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x
    y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y
    z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z
    w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w
    return Quaternion(x=x, y=y, z=z, w=w)

def quaternion_inverse(q: Quaternion):
    return Quaternion(x=q1.x, y=q1.y, z=q1.z, w=-q1.w)

def quaternion_angle_difference(q1: Quaternion, q2: Quaternion, axis: str):
    """
    Computes the angle difference between two quaternions relative to a given axis.
    
    Parameters:
    q1 (Quaternion): The first quaternion.
    q2 (Quaternion): The second quaternion.
    axis (str): The axis relative to which to compute the angle difference. Can be 'x', 'y', or 'z'.
    
    Returns:
    float: The angle difference in radians.
    """
    
    # Find the quaternion representing the rotation from q1 to q2
    q = quaternion_multiply(q2, quaternion_inverse(q1))
    
    # print(q_np)
    # Convert the quaternion to a rotation matrix
    R = r_matrixFromQuaternion(q)

    # Find the unit vector along the given axis
    if axis.lower() == 'x' or axis.lower=="roll":
        v = np.array([1, 0, 0])
    elif axis.lower() == 'y' or axis.lower=="pitch":
        v = np.array([0, 1, 0])
    elif axis.lower() == 'z' or axis.lower=="yaw":
        v = np.array([0, 0, 1])
    else:
        raise ValueError("Axis must be 'x/roll', 'y/pitch', or 'z/yaw'.")

    # Find the direction of the rotated axis
    v_rotated = R.dot(v)

    # Compute the angle between the rotated axis and the original axis
    angle = np.arccos(np.dot(v, v_rotated))

    return angle*180/math.pi

def euler_2_quat(yaw,pitch,roll)-> Quaternion:
    # Convert to radians
    yaw, pitch, roll = radians(yaw), radians(pitch), radians(roll)
    q = Quaternion()
    q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return q


def transform_coordinates(xyzrpy_a, xyzrpy_b):
    # Inputs xyzrpy_a, xyzrpy_2
    # Returns combination of xyzrpy 
    i,j,k,r,p,y = xyzrpy_a
    i2,j2,k2,r2,p2,y2 = xyzrpy_b
    homo_a = tf.affines.compose(T=[i,j,k], R=tf.euler.euler2mat(r,p,y),Z=[1,1,1])
    homo_b = tf.affines.compose(T=[i2,j2,k2], R=tf.euler.euler2mat(r2,p2,y2), Z=[1,1,1])
    
    #Combine Both
    T,R,Z,S = tf.affines.decompose(np.dot(homo_b,homo_a))
    print(T)
    R_euler = tf.euler.mat2euler(R)
    return np.concatenate((T,R_euler))

def r_matrixFromQuaternion(q:Quaternion):
    return np.array([[1 - 2*q.y**2 - 2*q.z**2, 2*q.x*q.y - 2*q.w*q.z, 2*q.x*q.z + 2*q.w*q.y],
                     [2*q.x*q.y + 2*q.w*q.z, 1 - 2*q.x**2 - 2*q.z**2, 2*q.y*q.z - 2*q.w*q.x],
                     [2*q.x*q.z - 2*q.w*q.y, 2*q.y*q.z + 2*q.w*q.x, 1 - 2*q.x**2 - 2*q.y**2]])
    
def find_absolute_pos(relative_pos:Point, current_quaternion:Quaternion):
    R = r_matrixFromQuaternion(current_quaternion)
    rotated_relative_position = np.dot(R,np.array([relative_pos.x, relative_pos.y, relative_pos.z]))
    return rotated_relative_position

if __name__ == '__main__':
    # Define two quaternions
    q1 = euler_2_quat(100,0,179)
    q2 = euler_2_quat(0,0,0)
    
    print(quaternion_angle_difference(q1,q2,"x"))
    print(quaternion_angle_difference(q1,q2,"y"))
    print(quaternion_angle_difference(q1,q2,"z"))
    
    