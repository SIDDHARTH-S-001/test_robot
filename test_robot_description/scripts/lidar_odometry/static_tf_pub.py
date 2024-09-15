#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_transform():
    # Initialize the ROS node
    rospy.init_node('static_transform_publisher')

    # Create a static transform broadcaster
    static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Define the static transform
    static_transform = geometry_msgs.msg.TransformStamped()

    # Set the frame names
    static_transform.header.frame_id = "odom"
    static_transform.child_frame_id = "odom_new"

    # Set the transform translation (x, y, z)
    static_transform.transform.translation.x = 0.0  # Set the translation in meters
    static_transform.transform.translation.y = 0.0  # Set the translation in meters
    static_transform.transform.translation.z = 0.0  # Set the translation in meters

    # Set the transform rotation (x, y, z, w)
    static_transform.transform.rotation.x = 0.0  # Set the rotation as quaternion
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    # Set the timestamp
    static_transform.header.stamp = rospy.Time.now()

    # Broadcast the static transform
    static_transform_broadcaster.sendTransform(static_transform)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_transform()
    except rospy.ROSInterruptException:
        pass
