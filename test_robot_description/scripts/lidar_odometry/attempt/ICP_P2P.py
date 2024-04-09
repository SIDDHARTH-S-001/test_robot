#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.point_cloud2 import read_points
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class LidarICP:
    def __init__(self):
        rospy.init_node('lidar_icp_node', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_pub = rospy.Publisher('/updated_pose', PoseStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.prev_scan = None
        self.odom_icp_pose = PoseStamped()  # Initialize odom_icp pose to zeros
        self.odom_icp_pose.header.frame_id = "odom_icp"
        self.H_mat = np.identity(4)

    def scan_callback(self, scan_msg):
        current_scan = self.laser_scan_to_point_cloud(scan_msg) # current_scan is an array of points        
        if self.prev_scan is not None:
            # Apply ICP
            H, R, t = self.point_to_point_icp(self.prev_scan, current_scan)            
            # Update pose
            self.update_pose(H, R, t)
        
        # Save the current scan for the next iteration
        self.prev_scan = current_scan

    def laser_scan_to_point_cloud(self, scan_msg):
        # Extract points from LaserScan message
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        # Convert polar coordinates to Cartesian coordinates
        points = []
        for i in range(len(ranges)):
            if ranges[i] < scan_msg.range_max:
                x = ranges[i] * np.cos(angles[i])
                y = ranges[i] * np.sin(angles[i])
                points.append([x, y])

        return np.array(points)
    
    def point_to_point_icp(self, source, target):
        # Use Nearest Neighbors to find correspondences
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(target)
        distances, indices = nbrs.kneighbors(source)
        # Extract matched points
        matched_target = target[indices.flatten()]        
        # Compute the transformation
        H, R, t = self.calculate_icp_transformation(source, matched_target) # prev_scan is the source and current_scan is target

        return H, R, t
    
    def calculate_icp_transformation(self, source, target):
        # Calculate the transformation between source and target using SVD
        # prev_scan is the source and current_scan is target
        mean_source = np.mean(source, axis=0)
        mean_target = np.mean(target, axis=0)
        centered_source = source - mean_source
        centered_target = target - mean_target
        shape_val = source.shape[1] # get number of dimensions
        W = np.dot(centered_source.T, centered_target)
        U, _, Vt = np.linalg.svd(W)
        R_val = np.dot(Vt.T, U.T)
        # special reflection case
        if np.linalg.det(R_val) < 0:
            Vt[shape_val-1,:] *= -1
            R_val = np.dot(Vt.T, U.T)
        t = mean_target - np.dot(R_val, mean_source)
        

        R = np.zeros((3, 3))
        R[:2, :2] = R_val
        R[2, 2] = 1

        H = np.identity(4) # homogeneous transformation matrix
        H[:3, :3] = R
        H[:2, 3] = t
        H[3, 3] = 1

        # print('H: ', np.round(H, 3))

        return np.round(H, 3),np.round(R, 3), np.round(t, 3)
    
    def update_pose(self,H, R, t):
        # Update the pose with the ICP transformation
        self.H_mat *= H
        trans = np.zeros((3, 1))
        trans[:3, 0] = H[:3, 3]
        Rot = H[:3, :3]
        # print(trans.shape, Rot.shape)
        self.odom_icp_pose.pose.position.x = trans[0][0]
        self.odom_icp_pose.pose.position.y = trans[1][0]
        updated_yaw = np.arctan2(Rot[1, 0], Rot[0, 0])  # Extracting rotation from the transformation matrix
        q = quaternion_from_euler(0, 0, updated_yaw)
        self.odom_icp_pose.pose.orientation.x = q[0]
        self.odom_icp_pose.pose.orientation.y = q[1]
        self.odom_icp_pose.pose.orientation.z = q[2]
        self.odom_icp_pose.pose.orientation.w = q[3]

        # Publish the updated pose
        self.pose_pub.publish(self.odom_icp_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lidar_icp = LidarICP()

    lidar_icp.run()
