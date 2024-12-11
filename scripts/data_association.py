#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from velodyne_msgs.msg import VelodyneScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageToPc:
    def __init__(self):
        rospy.init_node('image_to_pc', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/left/camera/image_raw', Image, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', VelodyneScan, self.pointcloud_callback)

        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.image = None
        self.image_path = os.path.expanduser('/home/uav/catkin_ws/src/toh_detector/data/image.jpg')
        self.image = None

    def image_callback(self, data):
        print("Image received")
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imwrite(self.image_path, self.image)
        rospy.loginfo('Image saved to %s', self.image_path)
    
    def pointcloud_callback(self, data):
        print("Pointcloud received")
        points = np.array(data.points)
        print(points)   

    # fucntion to find the correspondance betwen 

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('Image', self.image)
                cv2.waitKey(1)
            rate.sleep()

if __name__ == '__main__':
    image_to_pc = ImageToPc()
    image_to_pc.run()