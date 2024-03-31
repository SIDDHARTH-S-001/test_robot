#!/usr/bin/env python3

# This is for a two wheel differential drive robot
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import JointState
from math import cos, sin

class SimpleController(object):
    def __init__(self, wheel_radius, wheel_sepration):
        # rospy.loginfo("Wheel Radius (r): ", wheel_radius)
        # rospy.loginfo("Wheel Sepration (s): ", wheel_sepration)

        self.w_rad = wheel_radius
        self.w_sep = wheel_sepration
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = rospy.Time.now()

        # Initial Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.right_cmd_pub = rospy.Publisher("wheel_right_controller/command", Float64, queue_size=10)
        self.left_cmd_pub = rospy.Publisher("wheel_left_controller/command", Float64, queue_size=10)

        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callbak)

        self.speed_conversion_matrix = np.array([wheel_radius/2, wheel_radius/2], 
                                                [wheel_radius/wheel_sepration, -wheel_radius/wheel_sepration])
        
        rospy.loginfo("Conversion Matrix is: ", self.speed_conversion_matrix)     

    def vel_callback(self, msg):
        robot_speed = np.array([msg.linear.x], 
                               [msg.angular.z])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_matrix), robot_speed)
        right_speed = Float64(wheel_speed[0, 0])
        left_speed = Float64([1, 0])

        self.right_cmd_pub.publish(right_speed)
        self.left_cmd_pub.publish(left_speed)

    def joint_state_callbak(self, msg):
        dp_left = msg.position[0] - self.left_wheel_prev_pos
        dp_right = msg.position[1] - self.right_wheel_prev_pos
        dt = (msg.header.stamp - self.prev_time).to_sec()

        self.left_wheel_prev_pos = msg.position[0]
        self.right_wheel_prev_pos = msg.position[1]
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

        rospy.loginfo("Linear: %f  angular: %f x: %f y: %f theta: %f", linear_vel, angular_vel, self.x, self.y, self.theta)


if __name__=="__main__":
    rospy.init_node("Simple_Controller_Node", anonymous=True)
    # wheel_radius = rospy.get_param("~wheel_radius")
    # wheel_sepration = rospy.get_param("~wheel_sepration")
    wheel_radius = 0.05
    wheel_sepration = 0.3
    controller = SimpleController(wheel_radius, wheel_sepration)

    rospy.spin()



