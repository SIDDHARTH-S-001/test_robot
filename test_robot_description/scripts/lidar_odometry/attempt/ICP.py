#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from sklearn.neighbors import NearestNeighbors
import tf2_ros
from sensor_msgs.msg import LaserScan
import math
from tf.transformations import quaternion_from_euler

class ICPLocalization:
    def __init__(self):
        rospy.init_node('lidar_icp_node', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_pub = rospy.Publisher('/updated_pose', PoseStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.prev_scan = None

    def scan_callback(self, scan_msg):
        current_scan = self.laser_scan_to_point_cloud(scan_msg) # current_scan is an array of points
        
        if self.prev_scan is not None:
            # Apply ICP
            T, _, _ = self.icp(current_scan, self.prev_scan)
            print(T)
            self.update_pose(T)
        
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

    def best_fit_transform(self, source, target):
        '''
        Calculates the least-squares best-fit transform that maps corresponding source points to target points in m spatial dimensions
        Input:
        Source: Nxm numpy array of corresponding points
        Target: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps Source on to Target
        R: mxm rotation matrix
        t: mx1 translation vector
        '''
        # assert source.shape == target.shape

        # get number of dimensions
        m = source.shape[1]

        # translate points to their centroids
        centroid_source = np.mean(source, axis=0)
        centroid_target = np.mean(target, axis=0)
        mean_source = source - centroid_source
        mean_target = target - centroid_target

        # rotation matrix
        H = np.dot(mean_source.T, mean_target)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[m-1,:] *= -1
            R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_target.T - np.dot(R,centroid_source.T)

        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t

        return np.round(T, 3), R, t

    def nearest_neighbor(self, source, target):
        '''
        Find the nearest (Euclidean) neighbor in target for each point in source
        Input:
            source: Nxm array of points
            target: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: target indices of the nearest neighbor
        '''
        # assert source.shape == target.shape

        neigh = NearestNeighbors(n_neighbors=1, algorithm='ball_tree')
        neigh.fit(target)
        distances, indices = neigh.kneighbors(source, return_distance=True)
        
        return distances.ravel(), indices.ravel()

    def icp(self, source, target, init_pose=None, max_iterations=20, tolerance=0.001):
        '''
        The Iterative Closest Point method: finds best-fit transform that maps source points on to target points
        Input:
            Source: Nxm numpy array of source mD points
            Target: Nxm numpy array of destination mD point
            init_pose: (m+1)x(m+1) homogeneous transformation
            max_iterations: exit algorithm after max_iterations
            tolerance: convergence criteria
        Output:
            T: final homogeneous transformation that maps source on to target
            distances: Euclidean distances (errors) of the nearest neighbor
            i: number of iterations to converge
        '''
        # assert source.shape == target.shape

        # get number of dimensions
        m = source.shape[1]

        # make points homogeneous, copy them to maintain the originals
        src = np.ones((m+1, source.shape[0]))
        dst = np.ones((m+1, target.shape[0]))
        src[:m,:] = np.copy(source.T)
        dst[:m,:] = np.copy(target.T)

        # apply the initial pose estimation
        if init_pose is not None:
            src = np.dot(init_pose, src)

        prev_error = 0

        for i in range(max_iterations):
            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(src[:m,:].T, dst[:m,:].T)

            # compute the transformation between the current source and nearest destination points
            T,_,_ = self.best_fit_transform(src[:m,:].T, dst[:m,indices].T)

            # update the current source
            src = np.dot(T, src)

            # check error
            mean_error = np.mean(distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # calculate final transformation
        T,_,_ = self.best_fit_transform(source, src[:m,:].T)

        return T, distances, i
    
    def update_pose(self, T):
        # print(T.shape)
        pose = PoseStamped()
        pose.header.frame_id = "odom_icp"
        pose.pose.position.x = T[0][2]
        pose.pose.position.y = T[1][2]
        pose.pose.position.x = 0.0

        yaw = math.atan2(T[1][0], T[0][0])
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)

if __name__ == "__main__":
    icp_localization = ICPLocalization()
    rospy.spin()
