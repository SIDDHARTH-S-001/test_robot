#!/usr/bin/env python3

# This is for a two wheel differential drive robot
import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from math import cos, sin
from nav_msgs.msg import Odometry
import tf_conversions
from tf2_ros import TransformBroadcaster

class NoisyController(object):
    def __init__(self, wheel_radius, wheel_sepration):
        self.w_rad = wheel_radius
        self.w_sep = wheel_sepration
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = rospy.Time.now()

        # Initial Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.odom_br = TransformBroadcaster()
        self.tf_stamped = TransformStamped()
        self.tf_stamped.header.frame_id = "odom"
        self.tf_stamped.child_frame_id = "base_footprint_noisy"

        self.odom_pub = rospy.Publisher("test_robot/odom_noisy", Odometry, queue_size=10)

        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callbak)  

    def joint_state_callbak(self, msg):
        wheel_encoder_left = msg.position[0] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[1] + np.random.normal(0, 0.005)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos
        dt = (msg.header.stamp - self.prev_time).to_sec()

        self.left_wheel_prev_pos = wheel_encoder_left
        self.right_wheel_prev_pos = wheel_encoder_right
        self.prev_time = msg.header.stamp

        phi_left = dp_left/dt
        phi_right = dp_right/dt

        linear_vel = (self.w_rad / 2) * (phi_right + phi_left)
        angular_vel = (self.w_rad / self.w_sep) * (phi_right - phi_left)

        ds = (self.w_rad / 2) * (dp_right + dp_left) 
        dtheta = (self.w_rad / self.w_sep) * (dp_right - dp_left)

        self.theta += dtheta
        self.x += ds * cos(self.theta)
        self.y = ds * sin(self.theta)

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(self.odom_msg)

        self.tf_stamped.transform.translation.x = self.x
        self.tf_stamped.transform.translation.y = self.y
        self.tf_stamped.transform.rotation.x = q[0]
        self.tf_stamped.transform.rotation.y = q[1]
        self.tf_stamped.transform.rotation.z = q[2]
        self.tf_stamped.transform.rotation.w = q[3]
        self.tf_stamped.header.stamp = rospy.Time.now()

        self.odom_br.sendTransform(self.tf_stamped)

        # rospy.loginfo("Linear: %f  angular: %f x: %f y: %f theta: %f", linear_vel, angular_vel, self.x, self.y, self.theta)

if __name__=="__main__":
    rospy.init_node("Simple_Controller_Node", anonymous=True)
    # wheel_radius = rospy.get_param("~wheel_radius")
    # wheel_sepration = rospy.get_param("~wheel_sepration")
    wheel_radius = 0.05
    wheel_sepration = 0.3
    controller = NoisyController(wheel_radius, wheel_sepration)

    rospy.spin()



