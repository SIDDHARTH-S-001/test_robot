#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
import math

odom_data_pub = None
odomNew = Odometry()
odomOld = Odometry()

initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592

TICKS_PER_REVOLUTION = 3960/4
WHEEL_RADIUS = 0.07
WHEEL_BASE = 0.387908
TICKS_PER_METER = TICKS_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS)

displacementLeft = 0
displacementRight = 0

initialPoseReceived = False

def set_initial_2d(msg):
    global odomOld, initialPoseReceived
    odomOld.pose.pose.position.x = msg.pose.position.x
    odomOld.pose.pose.position.y = msg.pose.position.y
    odomOld.pose.pose.orientation.z = msg.pose.orientation.z
    initialPoseReceived = True

def calc_left(leftCount):
    global displacementLeft
    leftTicks = leftCount.data
    displacementLeft = leftTicks / TICKS_PER_METER

def calc_right(rightCount):
    global displacementRight
    rightTicks = rightCount.data
    displacementRight = rightTicks / TICKS_PER_METER

def publish_quat():
    global odomNew, odom_data_pub_quat
    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)

    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z

    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            quatOdom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            quatOdom.pose.covariance[i] += 0.1
        else:
            quatOdom.pose.covariance[i] = 0

    odom_data_pub_quat.publish(quatOdom)

def update_odom():
    global displacementLeft, displacementRight, odomNew, odomOld
    cycleDistance = (displacementRight + displacementLeft) / 2
    cycleAngle = math.asin((displacementRight - displacementLeft) / WHEEL_BASE)
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI

    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y) or math.isnan(odomNew.pose.pose.position.z):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI

    odomNew.header.stamp = rospy.Time.now()
    odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
    odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())

    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = odomNew.header.stamp

    odom_data_pub.publish(odomNew)

def main():
    global odomNew, odomOld, odom_data_pub, odom_data_pub_quat

    odomNew.header.frame_id = "odom"
    odomNew.pose.pose.position.z = 0
    odomNew.pose.pose.orientation.x = 0
    odomNew.pose.pose.orientation.y = 0
    odomNew.twist.twist.linear.x = 0
    odomNew.twist.twist.linear.y = 0
    odomNew.twist.twist.linear.z = 0
    odomNew.twist.twist.angular.x = 0
    odomNew.twist.twist.angular.y = 0
    odomNew.twist.twist.angular.z = 0
    odomOld.pose.pose.position.x = initialX
    odomOld.pose.pose.position.y = initialY
    odomOld.pose.pose.orientation.z = initialTheta

    rospy.init_node('diff_drive_odom_pub')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber('right_count', Int16, calc_right, queue_size=10)
    rospy.Subscriber('left_count', Int16, calc_left, queue_size=10)
    rospy.Subscriber('initial_2d', PoseStamped, set_initial_2d, queue_size=1)

    odom_data_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)
    odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if initialPoseReceived:
            update_odom()
            publish_quat()
        rate.sleep()

if __name__ == '__main__':
    main()
