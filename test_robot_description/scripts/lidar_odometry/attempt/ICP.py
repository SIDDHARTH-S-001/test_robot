#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from simpleicp import SimpleICP, PointCloud

class ICPBasedLocalization:
    def __init__(self):
        rospy.init_node('icp_localization')

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_pub = rospy.Publisher('/estimated_odometry', Odometry, queue_size=10)

        self.previous_scan = None
        self.current_scan = None

    def scan_callback(self, scan_msg):
        if self.previous_scan is None:
            self.previous_scan = np.array(scan_msg.ranges)
            return

        self.current_scan = np.array(scan_msg.ranges)

        # Perform ICP
        transformation, _, _ = SimpleICP(self.previous_scan, self.current_scan)

        # Extract translation
        x = transformation[0, 2]
        y = transformation[1, 2]

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        self.odom_pub.publish(odom_msg)

        # Update previous scan
        self.previous_scan = self.current_scan

if __name__ == '__main__':
    try:
        icp_localization = ICPBasedLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
