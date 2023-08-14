#!/usr/bin/env python3 
import numpy as np
import quaternion
from transforms3d.euler import euler2quat, quat2euler
import math
from math import radians
import transforms3d as tf   

def list_to_quat_arr(q):
    if len(q) != 3:
        assert ValueError("Bro your list needs to have 4 Values inserted")
    return np.quaternion(q[0],q[1],q[2],q[3])

def quaternion_angle_difference(q1, q2, axis):
    """
    Computes the angle difference between two quaternions relative to a given axis.
    
    Parameters:
    q1 (numpy.quaternion): The first quaternion.
    q2 (numpy.quaternion): The second quaternion.
    axis (str): The axis relative to which to compute the angle difference. Can be 'x', 'y', or 'z'.
    
    Returns:
    float: The angle difference in radians.
    """
    
    # Find the quaternion representing the rotation from q1 to q2
    q = q2 * q1.inverse()

    # Convert the quaternion to a rotation matrix
    R = np.array([[1 - 2*q.y**2 - 2*q.z**2, 2*q.x*q.y - 2*q.w*q.z, 2*q.x*q.z + 2*q.w*q.y],
                  [2*q.x*q.y + 2*q.w*q.z, 1 - 2*q.x**2 - 2*q.z**2, 2*q.y*q.z - 2*q.w*q.x],
                  [2*q.x*q.z - 2*q.w*q.y, 2*q.y*q.z + 2*q.w*q.x, 1 - 2*q.x**2 - 2*q.y**2]])

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

def euler_2_quat(x,y,z):
    yaw, pitch, roll = radians(x), radians(y), radians(z)
    qx,qy,qz,qw = euler2quat(yaw,pitch,roll)
    return np.quaternion(qw,qx,qy,qz)


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

if __name__ == '__main__':
    # Define two quaternions
    q1 = euler_2_quat(179,0,179)
    q2 = euler_2_quat(0,0,0)

    
    