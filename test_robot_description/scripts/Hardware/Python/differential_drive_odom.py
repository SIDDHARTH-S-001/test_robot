#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
import math

class DifferentialDriveOdom:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('diff_drive_odom_pub')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Parameters
        self.PI = 3.141592
        self.TICKS_PER_REVOLUTION = 3960 / 4
        self.WHEEL_RADIUS = 0.07
        self.WHEEL_BASE = 0.387908
        self.TICKS_PER_METER = self.TICKS_PER_REVOLUTION / (2 * self.PI * self.WHEEL_RADIUS)

        # State variables
        self.odom_data_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
        self.odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)
        self.odomNew = Odometry()
        self.odomOld = Odometry()
        self.initialPoseReceived = False
        self.displacementLeft = 0
        self.displacementRight = 0

        # Initial pose
        self.initialX = 0.0
        self.initialY = 0.0
        self.initialTheta = 0.00000000001

        # Subscribe to wheel encoder count topics
        rospy.Subscriber('right_count', Int16, self.calc_right, queue_size=10)
        rospy.Subscriber('left_count', Int16, self.calc_left, queue_size=10)

        # Subscribe to initial pose topic
        rospy.Subscriber('initial_2d', PoseStamped, self.set_initial_2d, queue_size=1)

    def set_initial_2d(self, msg):
        self.odomOld.pose.pose.position.x = msg.pose.position.x
        self.odomOld.pose.pose.position.y = msg.pose.position.y
        self.odomOld.pose.pose.orientation.z = msg.pose.orientation.z
        self.initialPoseReceived = True

    def calc_left(self, leftCount):
        self.displacementLeft = leftCount.data / self.TICKS_PER_METER

    def calc_right(self, rightCount):
        self.displacementRight = rightCount.data / self.TICKS_PER_METER

    def publish_quat(self):
        q = quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)

        quatOdom = Odometry()
        quatOdom.header.stamp = self.odomNew.header.stamp
        quatOdom.header.frame_id = "odom"
        quatOdom.child_frame_id = "base_link"
        quatOdom.pose.pose.position.x = self.odomNew.pose.pose.position.x
        quatOdom.pose.pose.position.y = self.odomNew.pose.pose.position.y
        quatOdom.pose.pose.position.z = self.odomNew.pose.pose.position.z
        quatOdom.pose.pose.orientation.x = q[0]
        quatOdom.pose.pose.orientation.y = q[1]
        quatOdom.pose.pose.orientation.z = q[2]
        quatOdom.pose.pose.orientation.w = q[3]
        quatOdom.twist.twist.linear.x = self.odomNew.twist.twist.linear.x
        quatOdom.twist.twist.linear.y = self.odomNew.twist.twist.linear.y
        quatOdom.twist.twist.linear.z = self.odomNew.twist.twist.linear.z
        quatOdom.twist.twist.angular.x = self.odomNew.twist.twist.angular.x
        quatOdom.twist.twist.angular.y = self.odomNew.twist.twist.angular.y
        quatOdom.twist.twist.angular.z = self.odomNew.twist.twist.angular.z

        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                quatOdom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                quatOdom.pose.covariance[i] += 0.1
            else:
                quatOdom.pose.covariance[i] = 0

        self.odom_data_pub_quat.publish(quatOdom)

    def update_odom(self):
        cycleDistance = (self.displacementRight + self.displacementLeft) / 2
        cycleAngle = math.asin((self.displacementRight - self.displacementLeft) / self.WHEEL_BASE)
        avgAngle = cycleAngle / 2 + self.odomOld.pose.pose.orientation.z

        if avgAngle > self.PI:
            avgAngle -= 2 * self.PI
        elif avgAngle < -self.PI:
            avgAngle += 2 * self.PI

        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y) or math.isnan(self.odomNew.pose.pose.position.z):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        if self.odomNew.pose.pose.orientation.z > self.PI:
            self.odomNew.pose.pose.orientation.z -= 2 * self.PI
        elif self.odomNew.pose.pose.orientation.z < -self.PI:
            self.odomNew.pose.pose.orientation.z += 2 * self.PI

        self.odomNew.header.stamp = rospy.Time.now()
        self.odomNew.twist.twist.linear.x = cycleDistance / (self.odomNew.header.stamp.to_sec() - self.odomOld.header.stamp.to_sec())
        self.odomNew.twist.twist.angular.z = cycleAngle / (self.odomNew.header.stamp.to_sec() - self.odomOld.header.stamp.to_sec())

        self.odomOld.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.odomOld.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.odomOld.pose.pose.orientation.z = self.odomNew.pose.pose.orientation.z
        self.odomOld.header.stamp = self.odomNew.header.stamp

        self.odom_data_pub.publish(self.odomNew)

def main():
    differential_drive_odom = DifferentialDriveOdom()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if differential_drive_odom.initialPoseReceived:
            differential_drive_odom.update_odom()
            differential_drive_odom.publish_quat()
        rate.sleep()

if __name__ == '__main__':
    main()
