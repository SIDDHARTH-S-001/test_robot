#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def static_transform_publisher():
    rospy.init_node('static_transform_publisher')
    
    static_tf_publisher = tf2_ros.StaticTransformBroadcaster()
    
    # Create a TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the transform between "map" and "odom_1"
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform("map", "odom_1", rospy.Time(0), rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform between 'map' and 'odom_1'. Retrying...")
            rospy.sleep(1.0)
            continue
    
    # Once the transform is obtained, set up the static transform between "base_frame" and "base_link"
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_frame"
    static_transformStamped.child_frame_id = "base_link"

    # Set the translation and rotation from the transform between "map" and "odom_1"
    static_transformStamped.transform.translation = transform.transform.translation
    static_transformStamped.transform.rotation = transform.transform.rotation

    # Publish the static transform
    static_tf_publisher.sendTransform(static_transformStamped)

    rospy.spin()

if __name__ == '__main__':
    try:
        static_transform_publisher()
    except rospy.ROSInterruptException:
        pass
