#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def static_transform_publisher():
    rospy.init_node('static_transform_publisher')
    
    static_tf_publisher = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "odom_1"
    static_transformStamped.child_frame_id = "base_frame"

    # Set the translation
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    # Set the rotation
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    static_tf_publisher.sendTransform(static_transformStamped)

    rospy.spin()

if __name__ == '__main__':
    try:
        static_transform_publisher()
    except rospy.ROSInterruptException:
        pass
