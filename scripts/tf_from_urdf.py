#!/usr/bin/env python3

import os
import numpy as np
import rospy
import matplotlib.pyplot as plt
from pytransform3d.urdf import UrdfTransformManager
import tf2_ros
import tf
import geometry_msgs


def load_urdf_file(filename):
    tm = UrdfTransformManager()
    with open(filename, "r") as f:
        robot_urdf = f.read()
        tm.load_urdf(robot_urdf, mesh_path=None)
    return tm

def extract_transformation_from_urdf(tm):
    T_base_imu = tm.get_transform("base_link_old", "imu_link")
    T_imu_lidar = tm.get_transform("imu_link", "lidar_link")
    T_imu_stereoR = tm.get_transform("imu_link", "stereo_R_link")
    T_imu_stereoL = tm.get_transform("imu_link", "stereo_L_link")
    T_stereoR_stereoROptical = tm.get_transform("stereo_R_link", "stereo_R_optical_link")
    T_stereoL_stereoLOptical = tm.get_transform("stereo_L_link", "stereo_L_optical_link")
    T_imu_multispectral = tm.get_transform("imu_link", "multispectral_link")
    return T_base_imu, T_imu_lidar, T_imu_stereoR, T_imu_stereoL, T_stereoR_stereoROptical, T_stereoL_stereoLOptical, T_imu_multispectral

def convert_to_tf(T, source, target):
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = source
    static_transformStamped.child_frame_id = target
    quat = tf.transformations.quaternion_from_matrix(T)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    static_transformStamped.transform.translation.x = T[0, 3]
    static_transformStamped.transform.translation.y = T[1, 3]
    static_transformStamped.transform.translation.z = T[2, 3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.loginfo("Published static transform from {} to {}".format(source, target))
    rospy.sleep(1)
    

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')
    current_dir = os.path.dirname(os.path.realpath(__file__))
    filename = current_dir + "/../urdf/mapping_system_pretty.urdf"
    tm = load_urdf_file(filename)
    T_base_imu, T_imu_lidar, T_imu_stereoR, T_imu_stereoL, T_stereoR_stereoROptical, T_stereoL_stereoLOptical, T_imu_multispectral = extract_transformation_from_urdf(tm)
    link_names = ["base_link_old", "imu_link", "lidar_link", "stereo_R_link", "stereo_L_link", "stereo_R_optical_link", "stereo_L_optical_link", "multispectral_link"]
    convert_to_tf(T_base_imu, "base_link_old", "imu_link")
    convert_to_tf(T_imu_lidar, "imu_link", "lidar_link")
    # Find the transformation matrix between the lidar_link and stereo_L_optical_link
    T_lidar_stereoL = np.dot(np.linalg.inv(T_imu_lidar), T_imu_stereoL)
    T_lidar_stereo_L_optical = np.dot(T_lidar_stereoL, T_stereoL_stereoLOptical)
    convert_to_tf(T_lidar_stereo_L_optical, "lidar_link", "stereo_L_optical_link")
    # convert_to_tf(T_imu_stereoR, "imu_link", "stereo_R_link")
    # convert_to_tf(T_imu_stereoL, "imu_link", "stereo_L_link")
    # convert_to_tf(T_stereoR_stereoROptical, "stereo_R_link", "stereo_R_optical_link")
    # convert_to_tf(T_stereoL_stereoLOptical, "stereo_L_link", "stereo_L_optical_link")
    # convert_to_tf(T_imu_multispectral, "imu_link", "multispectral_link")
    rospy.spin()
    # Convert these transformation matrix tf format
