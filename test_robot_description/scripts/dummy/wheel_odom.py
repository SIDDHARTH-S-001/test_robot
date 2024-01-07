#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import math

odom_data_pub = None
odom_data_pub_quat = None
odomNew = Odometry()
odomOld = Odometry()

initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592

TICKS_PER_REVOLUTION = 3960/4
WHEEL_RADIUS = 0.07
WHEEL_BASE = 0.387908
# 2pir = perimeter = ticks / rev ===> 1 m =  
TICKS_PER_METER = TICKS_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS)

distanceLeft = 0
distanceRight = 0

initialPoseRecieved = False

def set_initial_2d(msg):
    global odomOld, initialPoseRecieved
    odomOld.pose.pose.position.x = msg.pose.position.x
    odomOld.pose.pose.position.y = msg.pose.position.y
    odomOld.pose.pose.orientation.z = msg.pose.orientation.z
    initialPoseRecieved = True

def calc_left(leftCount):
    global distanceLeft
    lastCountL = [0]
    if leftCount.data != 0 and lastCountL[0] != 0:
        leftTicks = leftCount.data - lastCountL[0]
        if leftTicks > 10000:
            leftTicks = 0 - (65535 - leftTicks)
        elif leftTicks < -10000:
            leftTicks = 65535 - leftTicks
        distanceLeft = leftTicks / TICKS_PER_METER
    lastCountL[0] = leftCount.data

def calc_right(rightCount):
    global distanceRight
    lastCountR = [0]
    if rightCount.data != 0 and lastCountR[0] != 0:
        rightTicks = rightCount.data - lastCountR[0]
        if rightTicks > 10000:
            distanceRight = (0 - (65535 - distanceRight)) / TICKS_PER_METER
        elif rightTicks < -10000:
            rightTicks = 65535 - rightTicks
        distanceRight = rightTicks / TICKS_PER_METER
    lastCountR[0] = rightCount.data

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
    global distanceLeft, distanceRight, odomNew, odomOld
    cycleDistance = (distanceRight + distanceLeft) / 2
    cycleAngle = math.asin((distanceRight - distanceLeft) / WHEEL_BASE)
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI

    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y) or math.isnan(
            odomNew.pose.pose.position.z):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI

    odomNew.header.stamp = rospy.Time.now()
    odomNew.twist.twist.linear.x = cycleDistance / (
                odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
    odomNew.twist.twist.angular.z = cycleAngle / (
                odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())

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

    rospy.init_node('ekf_odom_pub')

    rospy.Subscriber('right_ticks', Int16, calc_right, queue_size=100)
    rospy.Subscriber('left_ticks', Int16, calc_left, queue_size=100)
    rospy.Subscriber('initial_2d', PoseStamped, set_initial_2d, queue_size=1)

    odom_data_pub = rospy.Publisher('odom_data_euler', Odometry, queue_size=100)
    odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if initialPoseRecieved:
            update_odom()
            publish_quat()
        rospy.spinOnce()
        rate.sleep()

if __name__ == '__main__':
    main()
