#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def handle_goal(goal):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = goal.header.stamp
    goal_pose.pose.position.x = goal.pose.position.x
    goal_pose.pose.position.y = goal.pose.position.y
    goal_pose.pose.position.z = 0

    (_, _, yaw) = euler_from_quaternion([0, 0, goal.pose.orientation.z, goal.pose.orientation.w])
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal_pose.pose.orientation.x = 0
    goal_pose.pose.orientation.y = 0
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = 0

    pub1.publish(goal_pose)

def handle_initial_pose(pose):
    init_pose = PoseStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = pose.header.stamp
    init_pose.pose.position.x = pose.pose.pose.position.x
    init_pose.pose.position.y = pose.pose.pose.position.y
    init_pose.pose.position.z = 0

    (_, _, yaw) = euler_from_quaternion([0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w])
    quat = quaternion_from_euler(0, 0, yaw)
    init_pose.pose.orientation.x = 0
    init_pose.pose.orientation.y = 0
    init_pose.pose.orientation.z = quat[2]
    init_pose.pose.orientation.w = 0

    pub2.publish(init_pose)

if __name__ == '__main__':
    rospy.init_node('rviz_2d_pose')
    pub1 = rospy.Publisher('goal_2d', PoseStamped, queue_size=0)
    pub2 = rospy.Publisher('initial_2d', PoseStamped, queue_size=0)
    sub1 = rospy.Subscriber('move_base_simple/goal', PoseStamped, handle_goal, queue_size=0)
    sub2 = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, handle_initial_pose, queue_size=0)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep()
