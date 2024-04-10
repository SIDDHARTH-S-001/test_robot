#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class ICPBasedLocalization:
    def __init__(self):
        rospy.init_node('icp_localization')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=10)
        self.prev_cloud = None

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        # Convert polar coordinates to Cartesian coordinates
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        points = np.array([[r * np.cos(theta), r * np.sin(theta), 0] for r, theta in zip(ranges, angles)])
        # Create PointCloud object
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        if self.prev_cloud is not None:
            # Perform ICP registration
            reg_p2p = o3d.pipelines.registration.registration_icp(
                self.prev_cloud, cloud, 10, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6)
            )

            # Update previous cloud
            self.prev_cloud = o3d.geometry.PointCloud(np.asarray(cloud.points) @ reg_p2p.transformation[:3, :3].T + reg_p2p.transformation[:3, 3])

            # Extract rotation angles
            R = reg_p2p.transformation[:3, :3]
            roll, pitch, yaw = euler_from_quaternion(o3d.geometry.OrientationRotationMatrixToQuaternion(R))

            # Publish estimated pose
            estimated_pose = PoseStamped()
            estimated_pose.header.stamp = rospy.Time.now()
            estimated_pose.pose.position.x = reg_p2p.transformation[0, 3]
            estimated_pose.pose.position.y = reg_p2p.transformation[1, 3]
            estimated_pose.pose.position.z = reg_p2p.transformation[2, 3]
            estimated_pose.pose.orientation.x = roll
            estimated_pose.pose.orientation.y = pitch
            estimated_pose.pose.orientation.z = yaw
            self.pose_pub.publish(estimated_pose)
        else:
            self.prev_cloud = cloud

if __name__ == '__main__':
    try:
        icp_localization = ICPBasedLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
