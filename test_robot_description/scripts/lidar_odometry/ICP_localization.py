import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.point_cloud2 import read_points
from tf.transformations import quaternion_from_matrix
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class LidarICP:
    def __init__(self):
        rospy.init_node('lidar_icp_node', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_pub = rospy.Publisher('/updated_pose', PoseStamped, queue_size=10)
        self.prev_scans = []
        self.cumulative_transform = np.eye(4)  # Initialize cumulative transformation as 4x4 identity matrix

    def scan_callback(self, scan_msg):
        current_scan = self.laser_scan_to_point_cloud(scan_msg)

        current_scan = self.box_filter(current_scan)

        if len(self.prev_scans) >= 5:
            self.prev_scans.pop(0)

        if self.prev_scans:
            R_avg, T_avg = self.average_icp_transformations(current_scan)

            # Increment the cumulative transformation
            self.cumulative_transform = np.dot(np.vstack([np.hstack([R_avg, T_avg[:, np.newaxis]]), [0, 0, 0, 1]]), self.cumulative_transform)

            # Update pose with the cumulative transformation
            self.update_pose(self.cumulative_transform)

        self.prev_scans.append(current_scan)

    def laser_scan_to_point_cloud(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        points = []
        for i in range(len(ranges)):
            if ranges[i] < scan_msg.range_max:
                x = ranges[i] * np.cos(angles[i])
                y = ranges[i] * np.sin(angles[i])
                z = 0  # Since it's a 2D scan
                points.append([x, y, z])

        return np.array(points)

    def box_filter(self, points, box_size=3):
        filtered_points = []
        for i in range(len(points)):
            x, y, z = points[i]
            if i < box_size or i >= len(points) - box_size:
                filtered_points.append([x, y, z])
            else:
                mean_x = np.mean([p[0] for p in points[i - box_size:i + box_size + 1]])
                mean_y = np.mean([p[1] for p in points[i - box_size:i + box_size + 1]])
                mean_z = np.mean([p[2] for p in points[i - box_size:i + box_size + 1]])
                filtered_points.append([mean_x, mean_y, mean_z])
        return np.array(filtered_points)

    def point_to_point_icp(self, source, target):
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(target)
        distances, indices = nbrs.kneighbors(source)
        matched_target = target[indices.flatten()]

        R, T = self.calculate_icp_transformation(source, matched_target)

        return R, T

    def calculate_icp_transformation(self, source, target):
        mean_source = np.mean(source, axis=0)
        mean_target = np.mean(target, axis=0)

        centered_source = source - mean_source
        centered_target = target - mean_target

        W = np.dot(centered_source.T, centered_target)

        U, _, Vt = np.linalg.svd(W)

        R = np.dot(Vt.T, U.T)
        T = mean_target - np.dot(R, mean_source)

        return R, T

    def average_icp_transformations(self, current_scan):
        R_sum = np.zeros((3, 3))
        T_sum = np.zeros(3)

        for prev_scan in self.prev_scans:
            R, T = self.point_to_point_icp(prev_scan, current_scan)
            R_sum += R
            T_sum += T

        R_avg = R_sum / len(self.prev_scans)
        T_avg = T_sum / len(self.prev_scans)

        return R_avg, T_avg

    def update_pose(self, transformation_matrix):
        updated_pose = PoseStamped()
        updated_pose.header.frame_id = "base_link"  # Change to your robot's frame
        updated_pose.header.stamp = rospy.Time.now()

        translation = transformation_matrix[:3, 3]
        quaternion = quaternion_from_matrix(transformation_matrix)

        updated_pose.pose.position.x = translation[0]
        updated_pose.pose.position.y = translation[1]
        updated_pose.pose.position.z = translation[2]

        updated_pose.pose.orientation.x = quaternion[0]
        updated_pose.pose.orientation.y = quaternion[1]
        updated_pose.pose.orientation.z = quaternion[2]
        updated_pose.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(updated_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lidar_icp = LidarICP()
    lidar_icp.run()
