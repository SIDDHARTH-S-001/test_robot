#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np

class SimpleController(object):
    def __init__(self, wheel_radius, wheel_sepration):
        rospy.loginfo("Wheel Radius (r): ", wheel_radius)
        rospy.loginfo("Wheel Sepration (s): ", wheel_sepration)

        self.right_cmd_pub = rospy.Publisher("wheel_right_controller/command", Float64, queue_size=10)
        self.left_cmd_pub = rospy.Publisher("wheel_left_controller/command", Float64, queue_size=10)

        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

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


if __name__=="__main__":
    rospy.init_node("Simple_Controller_Node", anonymous=True)
    wheel_radius = rospy.get_param("~wheel_radius")
    wheel_sepration = rospy.get_param("~wheel_sepration")
    controller = SimpleController(wheel_radius, wheel_sepration)

    rospy.spin()



