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
        self.global_map_pub = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)
        self.local_map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=10)

        self.map_resolution = 0.1  # meters per pixel
        self.map_width = 200        # in pixels
        self.map_height = 200       # in pixels
        self.map_origin_x = -10     # meters
        self.map_origin_y = -10     # meters

        self.global_map_data = np.zeros((self.map_width, self.map_height), dtype=np.int8)
        self.local_map_data = np.zeros((self.map_width, self.map_height), dtype=np.int8)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.num_local_maps = 5
        self.recent_local_maps = []

    def scan_callback(self, scan_msg):
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))

        # Initialize local map for current scan
        local_map = np.zeros((self.map_width, self.map_height), dtype=np.int8)

        for i, r in enumerate(scan_msg.ranges):
            if not math.isinf(r):
                x = int((r * math.cos(angles[i]) - self.map_origin_x) / self.map_resolution)
                y = int((r * math.sin(angles[i]) - self.map_origin_y) / self.map_resolution)

                # Check if point is within local map bounds
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    # Occupied cell
                    local_map[x, y] = 100
                else:
                    # Ignore out-of-bounds points
                    rospy.logwarn("Scan point out of local map bounds.")
                
        # Append the current local map to recent local maps
        self.recent_local_maps.append(local_map)

        # Keep only the most recent local maps
        if len(self.recent_local_maps) > self.num_local_maps:
            self.recent_local_maps.pop(0)

        # Compute the average of recent local maps
        if self.recent_local_maps:
            self.local_map_data = np.mean(self.recent_local_maps, axis=0)

        # Update global map
        self.update_global_map()

    def update_global_map(self):
        # Calculate the offset indices based on the robot's position in the global map
        x_offset = int((self.robot_x - self.map_origin_x) / self.map_resolution)
        y_offset = int((self.robot_y - self.map_origin_y) / self.map_resolution)

        # Ensure that the offset indices are within the bounds of the global map
        x_offset = max(0, min(x_offset, self.map_width - self.local_map_data.shape[0]))
        y_offset = max(0, min(y_offset, self.map_height - self.local_map_data.shape[1]))

        # Paste the local map data into the global map
        x_end = min(x_offset + self.local_map_data.shape[0], self.map_width)
        y_end = min(y_offset + self.local_map_data.shape[1], self.map_height)
        self.global_map_data[x_offset:x_end, y_offset:y_end] = self.local_map_data[:x_end-x_offset, :y_end-y_offset]

        # Fill unexplored cells with 0
        self.global_map_data[self.global_map_data != 100] = 0

        # Publish global map
        self.publish_global_map()

    def odom_callback(self, odom_msg):
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y

    def publish_global_map(self):
        global_map_msg = OccupancyGrid()
        global_map_msg.header.stamp = rospy.Time.now()
        global_map_msg.header.frame_id = 'map'
        global_map_msg.info.resolution = self.map_resolution
        global_map_msg.info.width = self.map_width
        global_map_msg.info.height = self.map_height
        global_map_msg.info.origin.position.x = self.map_origin_x
        global_map_msg.info.origin.position.y = self.map_origin_y
        global_map_msg.info.origin.position.z = 0.0
        global_map_msg.info.origin.orientation.x = 0.0
        global_map_msg.info.origin.orientation.y = 0.0
        global_map_msg.info.origin.orientation.z = 0.0
        global_map_msg.info.origin.orientation.w = 1.0
        global_map_msg.data = np.ravel(self.global_map_data).tolist()

        self.global_map_pub.publish(global_map_msg)

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
