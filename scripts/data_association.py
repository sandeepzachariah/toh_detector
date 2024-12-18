#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PixelToPointMapper:
    """Class to map a pixel coordinate to a 3D point in the point cloud.
    """

    def __init__(self):
        """Default constructor.
        """
        rospy.init_node('pixel_to_point_mapper', anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber('/left/camera/image_raw', \
                                        Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber('/left/camera/camera_info', \
                                    CameraInfo, self.camera_info_callback)
        self.pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, \
                                                self.pointcloud_callback)

        # Publisher 
        self.pub = rospy.Publisher('/highlighted_point', Marker, queue_size=10)

        # TF Buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()

        self.camera_info_received = False

        # Get pixel coordinates from parameters or set default values
        self.given_u = rospy.get_param('~pixel_u', 1224)
        self.given_v = rospy.get_param('~pixel_v', 1024)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.image = None

        # Output file for saving point cloud
        self.output_pc_filename = rospy.get_param('~output_filename', \
            '/home/uav/catkin_ws/src/toh_detector/temp/pointcloud.txt')
        self.image_filename = rospy.get_param('~image_filename', \
            '/home/uav/catkin_ws/src/toh_detector/temp/image.png')
        self.image_all_pts_filename = rospy.get_param('~image_all_pts_filename', \
            '/home/uav/catkin_ws/src/toh_detector/temp/image_all_pts.png')

    def image_callback(self, msg):
        """Callback function for the image subscriber.

        Args:
            msg (sensor_msgs.msg.Image): Image message
        """
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        """Callback function for the camera info subscriber.
        Sets the camera matrix and distortion coefficients.

        Args:
            msg (sensor_msgs.msg.CameraInfo): Camera info message
        """
        if not self.camera_info_received:
            # Extract camera intrinsic parameters
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)
            self.camera_info_received = True
            rospy.loginfo("Camera info received.")

    def pointcloud_callback(self, msg):
        """Callback function for point cloud subscriber.
        Triggers the pixel-to-point mapping.

        Args:
            msg (sensor_msgs.msg.PointCloud2): Point cloud message
        """
        # Check if camera info is received
        if not self.camera_info_received:
            rospy.logwarn("Camera info not received yet.")
            return
        
        # Check if image is received
        if self.image is None:
            rospy.logwarn("Image not received yet.")
            return

        # Transform the point cloud to the camera frame
        source_frame = msg.header.frame_id
        assert source_frame == "lidar_link"
        target_frame = 'stereo_L_optical_link'  

        try:
            # Get the transform from the lidar frame to the camera frame
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, \
                            source_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Transform lookup failed: %s", ex)
            return

        # Convert PointCloud2 message to a list of 3D points
        cloud_points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = [point[0], point[1], point[2], 1.0]  # Homogeneous coordinates
            cloud_points.append(pt)

        if not cloud_points:
            rospy.logwarn("No valid points in point cloud.")
            return

        # Convert the transform to a 4x4 matrix
        transform_mat = self.transform_to_matrix(transform_stamped.transform)

        # Transform points to the camera frame
        points_in_camera_frame = []
        for pt in cloud_points:
            pt_cam = np.dot(transform_mat, pt)
            if pt_cam[2] > 0:  # Only consider points in front of the camera
                points_in_camera_frame.append(pt_cam[:3])

        if not points_in_camera_frame:
            rospy.logwarn("No points in front of the camera after transformation.")
            return

        # Project 3D points onto the image plane
        object_points = np.array(points_in_camera_frame)
        rvec = np.zeros((3, 1))
        rvec = np.array([0.0, 0.0, 3.1417])
        tvec = np.zeros((3, 1))
        tvec = np.array([0.20, -0.2, 0.0])
        image_points, _ = cv2.projectPoints(object_points, rvec, tvec, \
                                    self.camera_matrix, self.dist_coeffs)

        # Find the 3D point corresponding to the given pixel coordinates
        min_distance = 100
        corresponding_point = None
        point_found = False
        no_points_inside_image = 0
        points_inside_image = []
        for idx, img_pt in enumerate(image_points):
            x = int(round(img_pt[0][0]))
            y = int(round(img_pt[0][1]))
                    
            # find the nearest point to the given pixel
            # make sure x and y valye are within the image size 
            if x < 0 or x >= self.image.shape[1] or y < 0 or y >= self.image.shape[0]:
                continue
            no_points_inside_image += 1
            points_inside_image.append((x, y))
            distance = np.sqrt((x - self.given_u)**2 + (y - self.given_v)**2)
            if distance < min_distance:
                min_distance = distance
                corresponding_point = object_points[idx]
                point_found = True
            
        rospy.loginfo("Number of points inside the image: %d", no_points_inside_image)

        #TODO: Visualize all the points that are inside the image
        # visualize the points that are inside the image
        self.visualize_points_inside_image(points_inside_image)

        if point_found:
            self.pointcloud_saver(msg)
            rospy.loginfo("Corresponding 3D point at pixel (%d, %d): x=%f, y=%f, z=%f with distance %f",
                          self.given_u, self.given_v,
                          corresponding_point[0], corresponding_point[1], corresponding_point[2], min_distance)
            
            # visualize the point cloud
            # transform the point back to the camera frame
            localized_point_lidar = np.dot(np.linalg.inv(transform_mat), np.array([corresponding_point[0], corresponding_point[1], corresponding_point[2], 1.0]))
            self.visualize(localized_point_lidar)
            self.publish_marker(localized_point_lidar[0], localized_point_lidar[1], localized_point_lidar[2])
            is_point_in_pc = self.cross_check_point_in_pc(points_in_camera_frame, corresponding_point)
            print("Is the point in the point cloud? ", is_point_in_pc)
            
            return 
        else:
            rospy.logwarn("No corresponding 3D point found for pixel (%d, %d).", self.given_u, self.given_v)

    def pointcloud_saver(self, cloud_msg):
        # Convert from PointCloud2 message to iterable of points
        points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Write points to file in ASCII format: x y z
        with open(self.output_pc_filename, 'w') as f:
            for p in points:
                f.write("{:.6f} {:.6f} {:.6f}\n".format(p[0], p[1], p[2]))
        
        rospy.loginfo("Pointcloud saved to %s" % self.output_pc_filename)


    def transform_to_matrix(self, transform):
        # Convert geometry_msgs/Transform to a 4x4 transformation matrix
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]

        trans_mat = tf.transformations.translation_matrix(translation)
        rot_mat = tf.transformations.quaternion_matrix(rotation)

        transform_mat = np.dot(trans_mat, rot_mat)
        return transform_mat

    def visualize_points_inside_image(self, points):
        # function to visualize the points that are inside the image
        if self.image is not None:
            image = self.image.copy()
            for pt in points:
                cv2.circle(image, pt, 5, (0, 255, 0), -1)
            # set the image window size
            # save the image 
            cv2.imwrite(self.image_all_pts_filename, image)
        else:
            rospy.logwarn("No image received yet.")
        return

    # function to visualze the image and the point cloud with the corresponding pixel and point marked
    def visualize(self, point):
        if self.image is not None:
            image = self.image.copy()
            cv2.circle(image, (self.given_u, self.given_v), 5, (0, 0, 255), -1)
            # set the image window size
            # save the image 
            cv2.imwrite(self.image_filename, image)
        else:
            rospy.logwarn("No image received yet.")
        
        # plot the point cloud
        pcd = o3d.io.read_point_cloud("/home/uav/catkin_ws/src/toh_detector/temp/pointcloud.txt", format='xyz')
        point = point[:3]
        highlighted_pcd, highlight_location = self.highlight_point_in_cloud(pcd, point)
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05    )
        sphere.translate(highlight_location)
        sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red sphere

        # Visualize
        o3d.visualization.draw_geometries([highlighted_pcd, sphere])
        return

    def highlight_point_in_cloud(self, pcd, query_point, highlight_color=[1.0, 1.0, 1.0]):
        # Build a KD-tree for fast nearest-neighbor search
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        # Query for the nearest neighbor
        [k, idx, _] = pcd_tree.search_knn_vector_3d(query_point, 1)
        # initialize the color array with blue color
        pcd.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 1.0] for _ in range(len(pcd.points))]))
        if k > 0:
            # idx[0] is the index of the closest point
            highlight_location = pcd.points[idx[0]]
        else:
            print("No nearest point found.")

        return pcd, highlight_location  


    def cross_check_point_in_pc(self, points, point):
        # function to cross check if the point is in the point cloud
        for pt in points:
            # within a threshold
            if np.linalg.norm(np.array(pt) - np.array(point)) < 0.1:
                return True
        return False

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "lidar_link"  # Same frame as your point cloud
        marker.header.stamp = rospy.Time.now()

        # Set the type of marker you want. For a sphere:
        marker.type = Marker.SPHERE

        # The point you want to highlight
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation should be valid, even though for a sphere it's not critical
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale of the sphere
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color (RGBA), values between 0 and 1. Here let's make it bright red:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set how long the marker should last before disappearing
        # 0 is forever
        marker.lifetime = rospy.Duration(0)

        # Publish at a set rate so it stays visible
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            marker.header.stamp = rospy.Time.now()
            self.pub.publish(marker)
            rate.sleep()
        

if __name__ == '__main__':
    try:
        mapper = PixelToPointMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
