#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import struct
from sensor_msgs.msg import Image
import numpy as np

np_img = None
h = 0
w = 0
sim =True

# define the image callback function to process the incoming image messages
def image_callback(msg):
    global np_img,w,h

    h = msg.height
    w = msg.width
    np_img = np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height, msg.width, -1)
    print(np_img.shape)

def point_cloud_callback(msg,pub):
    global np_img,w

    # Access the data field as a bytearray
    data = bytearray(msg.data)

    hix = 0
    wix = 0

     # Loop through the data and modify the RGB values
    for i in range(0, len(data), msg.point_step):
        # Calculate the offset for the RGB values
        rgb_offset = i + 16

        data[rgb_offset] = np_img[hix,wix,2]  # Blue channel
        data[rgb_offset + 1] = np_img[hix,wix,1]   # Green channel
        data[rgb_offset + 2] = np_img[hix,wix,0]   # red channel

        if wix < w-1:
            wix += 1
        else:
            hix += 1
            wix = 0

        # if hix < h-1:
        #     hix += 1
        # else:
        #     wix += 1
        #     hix = 0
    
    
    modified_data = np.asarray(data, dtype=np.uint8)
    msg.data = modified_data.tobytes()
    pub.publish(msg)

def main():
    rospy.init_node('point_cloud_subscriber')

    

    if sim:
        pub = rospy.Publisher('/realsense/depth/color/points2', PointCloud2, queue_size=10)
        rospy.Subscriber('/realsense/depth/color/points', PointCloud2, point_cloud_callback,pub)
        rospy.Subscriber('/object_detection_image/mask', Image, image_callback)
    else:
        pub = rospy.Publisher('/d435/depth/color/points2', PointCloud2, queue_size=10)
        rospy.Subscriber('/d435/depth/color/points', PointCloud2, point_cloud_callback,pub)
        rospy.Subscriber('/d435/color/image_raw', Image, image_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
