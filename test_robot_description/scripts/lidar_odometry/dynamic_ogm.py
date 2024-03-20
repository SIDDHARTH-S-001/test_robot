#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OccupancyGridMapping:
    def __init__(self):
        rospy.init_node('occupancy_grid_mapping')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        self.map_resolution = 0.1  # meters per pixel
        self.map_origin_x = -10     # meters
        self.map_origin_y = -10     # meters

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.maps = []  # List to store maps and corresponding robot poses

    def scan_callback(self, scan_msg):
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))

        map_data = np.zeros_like(self.maps[-1][0]) if self.maps else np.zeros((200, 200), dtype=np.int8)

        for i, r in enumerate(scan_msg.ranges):
            if not math.isinf(r):
                x = int((r * math.cos(angles[i]) - self.map_origin_x + self.robot_x) / self.map_resolution)
                y = int((r * math.sin(angles[i]) - self.map_origin_y + self.robot_y) / self.map_resolution)

                if 0 <= x < map_data.shape[0] and 0 <= y < map_data.shape[1]:
                    # Occupied cell
                    map_data[x, y] = 100

        self.maps.append((map_data, (self.robot_x, self.robot_y)))
        self.publish_map()
        self.publish_map_odom_tf()

    def odom_callback(self, odom_msg):
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y

        self.update_map_size()

    def update_map_size(self):
        # Calculate map dimensions based on robot's position
        max_x = max(abs(self.robot_x - self.map_origin_x), abs(self.map_origin_x - self.robot_x))
        max_y = max(abs(self.robot_y - self.map_origin_y), abs(self.map_origin_y - self.robot_y))

        # Convert to pixels
        self.map_width = int(max_x * 2 / self.map_resolution)
        self.map_height = int(max_y * 2 / self.map_resolution)

        # Initialize map data with zeros
        self.map_data = np.zeros((self.map_width, self.map_height), dtype=np.int8)

    def publish_map(self):
        for map_data, _ in self.maps:
            map_msg = OccupancyGrid()
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = 'map'
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = map_data.shape[0]
            map_msg.info.height = map_data.shape[1]
            map_msg.info.origin.position.x = self.map_origin_x
            map_msg.info.origin.position.y = self.map_origin_y
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.x = 0.0
            map_msg.info.origin.orientation.y = 0.0
            map_msg.info.origin.orientation.z = 0.0
            map_msg.info.origin.orientation.w = 1.0
            map_msg.data = np.ravel(map_data).tolist()

            self.map_pub.publish(map_msg)

    def publish_map_odom_tf(self):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "odom"

        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':
    try:
        occupancy_grid_mapping = OccupancyGridMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
