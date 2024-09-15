#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros

class LidarOdometry:
    def __init__(self):
        rospy.init_node('lidar_odometry')

        # Subscribers and publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_pub = rospy.Publisher('/odom_est', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Previous point cloud and pose
        self.prev_cloud = None
        self.prev_pose = Pose()

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud
        point_cloud = self.laser_scan_to_point_cloud(scan_msg)
        
        if self.prev_cloud is not None:
            # Perform ICP to align point clouds
            reg_icp = o3d.pipelines.registration.registration_icp(
                point_cloud, self.prev_cloud, 0.1,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            
            transformation = reg_icp.transformation
            
            # Extract translation and rotation
            translation = transformation[:3, 3]
            rotation = transformation[:3, :3]
            euler = self.rotation_matrix_to_euler_angles(rotation)
            
            # Update pose
            current_pose = Pose()
            current_pose.position.x = self.prev_pose.position.x + translation[0]
            current_pose.position.y = self.prev_pose.position.y + translation[1]
            current_pose.position.z = self.prev_pose.position.z + translation[2]
            current_pose.orientation.x = euler[0]
            current_pose.orientation.y = euler[1]
            current_pose.orientation.z = euler[2]
            current_pose.orientation.w = 1.0
            
            # Publish odometry
            self.publish_odometry(current_pose)
            
            # Update previous cloud and pose
            self.prev_cloud = point_cloud
            self.prev_pose = current_pose
        else:
            # Initialize previous cloud and pose
            self.prev_cloud = point_cloud
            self.prev_pose = Pose()

    def laser_scan_to_point_cloud(self, scan_msg):
        # Convert LaserScan to numpy array
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        x = scan_msg.ranges * np.cos(angles)
        y = scan_msg.ranges * np.sin(angles)
        
        # Create PointCloud
        points = np.vstack((x, y, np.zeros_like(x))).T
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        return point_cloud

    def publish_odometry(self, pose):
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom_new'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose
        self.odom_pub.publish(odom_msg)

        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'odom_new'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def rotation_matrix_to_euler_angles(self, R):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

if __name__ == '__main__':
    try:
        lidar_odometry = LidarOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
