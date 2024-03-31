#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=True)

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_publisher = rospy.Publisher('/trajectory', Path, queue_size=10)

        self.robot_trajectory = Path()
        self.robot_trajectory.header.frame_id = "odom" 

    def odom_callback(self, msg):
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # Extracting euler angles from quaternion
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Create a new pose stamped
        pose_stamped = PoseStamped()
        pose_stamped.pose.position = position
        pose_stamped.pose.orientation = orientation
        pose_stamped.header = msg.header

        # Append the pose to the trajectory
        self.robot_trajectory.poses.append(pose_stamped)
        print(self.robot_trajectory.poses[-1])

        # Publish the trajectory
        self.path_publisher.publish(self.robot_trajectory)

if __name__ == '__main__':
    try:
        trajectory_publisher = TrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
