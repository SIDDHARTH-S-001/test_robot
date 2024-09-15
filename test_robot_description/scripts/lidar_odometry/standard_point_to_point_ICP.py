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
        self.odom_pub = rospy.Publisher('/odom_est', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize previous point cloud and pose
        self.prev_cloud = None
        self.prev_pose = np.identity(3)  # 3x3 transformation matrix for 2D
        self.frame_skip = 5  # Process every 5th frame
        self.frame_count = 0

    def scan_callback(self, scan_msg):
        self.frame_count += 1

        # Only process every `frame_skip` frames
        if self.frame_count % self.frame_skip != 0:
            return

        # Step 1: Convert LaserScan to point cloud
        point_cloud = self.laser_scan_to_point_cloud(scan_msg)

        # Step 2: If no previous scan, store current scan and return
        if self.prev_cloud is None:
            self.prev_cloud = point_cloud
            return

        # Step 3-9: Perform ICP between previous and current point cloud using Euclidean distance
        transformation = self.icp(self.prev_cloud, point_cloud)

        # Step 10: Separate rotation and translation
        rotation, translation = transformation[:2, :2], transformation[:2, 2]

        # Apply rotation to the translation vector so that translation is applied in the rotated frame
        translation_global = np.dot(self.prev_pose[:2, :2], translation)

        # Step 11: Build the transformation matrix
        current_transform = np.identity(3)
        current_transform[:2, :2] = rotation  # Apply the rotation
        current_transform[:2, 2] = translation_global  # Apply the rotated translation in the global frame

        # Update the total pose
        self.prev_pose = np.dot(current_transform, self.prev_pose)

        # Step 12: Convert pose to odometry and publish
        self.publish_odometry(self.prev_pose)

        # Step 13: Store the current scan as the previous scan for the next iteration
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

    def icp(self, source, target, max_iterations=1000, tolerance=1e-4):
        """
        Perform Point-to-Point ICP between two 2D point clouds using Chamfer Distance.
        """
        prev_error = float('inf')
        transformation = np.identity(3)  # Initialize 3x3 2D transformation matrix

        for _ in range(max_iterations):
            # Step 3: Find correspondences (nearest neighbors)
            indices_source = self.find_correspondences(source, target)
            indices_target = self.find_correspondences(target, source)

            # Ensure that source and target correspondences have equal size
            if len(indices_source) != len(indices_target):
                min_size = min(len(indices_source), len(indices_target))
                indices_source = indices_source[:min_size]
                indices_target = indices_target[:min_size]

            # Step 4: Get matched points in target based on nearest neighbors
            matched_source = source  # Source points
            matched_target = target[indices_source]  # Matched points from target

            # Step 4 (continued): Compute centroids of matched points
            src_centroid = np.mean(matched_source, axis=0)
            tgt_centroid = np.mean(matched_target, axis=0)

            # Step 4 (continued): Center the points
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

            # Compute the Chamfer distance as error
            error_source_to_target = np.mean(np.min(np.linalg.norm(source[:, None] - target[None, :], axis=2), axis=1))
            error_target_to_source = np.mean(np.min(np.linalg.norm(target[:, None] - source[None, :], axis=2), axis=1))

            error = (error_source_to_target + error_target_to_source) / 2

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
        _, indices = tree.query(source, k=1)
        
        # Return the indices of the nearest neighbors
        return indices.flatten()

    def reject_outliers(self, distances):
        """
        Rejects outliers using the IQR method.
        """
        q1 = np.percentile(distances, 25)
        q3 = np.percentile(distances, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        # Return a mask of points that are within the IQR range
        return (distances >= lower_bound) & (distances <= upper_bound)

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
        odom_msg.header.frame_id = 'odom_new'
        odom_msg.child_frame_id = 'base_link'

        # Set the position and orientation
        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = 0  # Since it's 2D planar robot
        odom_msg.pose.pose.orientation.x = rotation[0]
        odom_msg.pose.pose.orientation.y = rotation[1]
        odom_msg.pose.pose.orientation.z = rotation[2]
        odom_msg.pose.pose.orientation.w = rotation[3]

        # Publish the odometry
        self.odom_pub.publish(odom_msg)

        # Broadcast the transformation
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'odom_new'
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
