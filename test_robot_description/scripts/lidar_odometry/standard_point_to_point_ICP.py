#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_matrix
from sklearn.neighbors import BallTree

class LidarICP:
    def __init__(self):
        rospy.init_node('lidar_icp')

        # ROS Subscribers and Publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_pub = rospy.Publisher('/odom_estimated', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize previous point cloud
        self.prev_cloud = None
        self.prev_pose = np.identity(3)  # 3x3 transformation matrix for 2D

    def scan_callback(self, scan_msg):
        # Step 1: Convert LaserScan to point cloud
        point_cloud = self.laser_scan_to_point_cloud(scan_msg)

        # Step 2: If no previous scan, store current scan and return
        if self.prev_cloud is None:
            self.prev_cloud = point_cloud
            return

        # Step 3-9: Perform ICP between previous and current point cloud
        transformation = self.icp(self.prev_cloud, point_cloud)

        # Step 10: Update the pose by applying the transformation
        self.prev_pose = np.dot(self.prev_pose, transformation)

        # Step 11: Convert pose to odometry and publish
        self.publish_odometry(self.prev_pose)

        # Step 12: Store the current scan as the previous scan for the next iteration
        self.prev_cloud = point_cloud

    def laser_scan_to_point_cloud(self, scan_msg):
        """
        Converts LaserScan data to a 2D point cloud
        """
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)
        valid_indices = np.isfinite(ranges)  # Filter out invalid ranges
        x = ranges[valid_indices] * np.cos(angles[valid_indices])
        y = ranges[valid_indices] * np.sin(angles[valid_indices])

        # Return Nx2 point cloud
        return np.vstack((x, y)).T

    def icp(self, source, target, max_iterations=50, tolerance=1e-5):
        """
        Perform Point-to-Point ICP between two 2D point clouds.
        """
        prev_error = float('inf')
        transformation = np.identity(3)  # Initialize 3x3 2D transformation matrix

        for _ in range(max_iterations):
            # Step 3: Find correspondences (nearest neighbors)
            indices = self.find_correspondences(source, target)

            # Step 4: Get matched points in target based on nearest neighbors
            matched_source = source  # The source points
            matched_target = target[indices]  # The corresponding points from the target

            # Step 4 (continued): Compute centroids of matched points
            src_centroid = np.mean(matched_source, axis=0)
            tgt_centroid = np.mean(matched_target, axis=0)

            # Center the points
            src_centered = matched_source - src_centroid
            tgt_centered = matched_target - tgt_centroid

            # Step 5: Compute cross-covariance matrix
            H = np.dot(src_centered.T, tgt_centered)

            # Step 6-7: Compute SVD and derive rotation
            U, _, Vt = np.linalg.svd(H)
            R = np.dot(Vt.T, U.T)

            # Step 8: Ensure proper rotation (reflection handling)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = np.dot(Vt.T, U.T)

            # Step 9: Compute translation
            t = tgt_centroid - np.dot(R, src_centroid)

            # Build 3x3 transformation matrix (homogeneous coordinates)
            current_transform = np.identity(3)
            current_transform[:2, :2] = R  # Rotation part
            current_transform[:2, 2] = t   # Translation part

            # Update total transformation
            transformation = np.dot(current_transform, transformation)

            # Apply transformation to source
            source = np.dot(source, R.T) + t

            # Compute the error
            error = np.mean(np.linalg.norm(source - matched_target, axis=1))
            if abs(prev_error - error) < tolerance:
                break
            prev_error = error

        # Return the final 3x3 transformation for 2D (homogeneous coordinates)
        return transformation

    def find_correspondences(self, source, target):
        """
        Find the nearest neighbors in `target` for each point in `source`
        using Ball Tree algorithm for efficient nearest-neighbor search.
        """
        # Build Ball Tree for the target point cloud
        tree = BallTree(target, leaf_size=40)

        # Query for the nearest neighbors in the target for each point in source
        distances, indices = tree.query(source, k=1)
        
        # Return the indices of the nearest neighbors
        return indices.flatten()

    def publish_odometry(self, pose_matrix):
        """
        Convert 3x3 pose matrix to Odometry message and publish it.
        """
        # Extract translation and rotation (as quaternion) from the pose matrix
        translation = pose_matrix[:2, 2]
        rotation_angle = np.arctan2(pose_matrix[1, 0], pose_matrix[0, 0])
        rotation = quaternion_from_matrix(np.array([[pose_matrix[0, 0], pose_matrix[0, 1], 0, 0],
                                                    [pose_matrix[1, 0], pose_matrix[1, 1], 0, 0],
                                                    [0, 0, 1, 0],
                                                    [0, 0, 0, 1]]))

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_link'

        # Set the position and orientation
        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = 0  # Since it's 2D planar
        odom_msg.pose.pose.orientation.x = rotation[0]
        odom_msg.pose.pose.orientation.y = rotation[1]
        odom_msg.pose.pose.orientation.z = rotation[2]
        odom_msg.pose.pose.orientation.w = rotation[3]

        # Publish the odometry
        self.odom_pub.publish(odom_msg)

        # Broadcast the transformation
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = 0  # Assuming 2D planar motion
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        lidar_icp = LidarICP()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
