#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class LidarOccupancyGridMapper:
    def __init__(self):
        rospy.init_node('lidar_occupancy_grid_mapper')

        self.map_resolution = 0.05  # meters per cell
        self.map_width = 800        # in cells
        self.map_height = 800       # in cells
        self.map_origin_x = -20.0   # in meters
        self.map_origin_y = -20.0   # in meters

        self.map = np.zeros((self.map_width, self.map_height), dtype=np.int8)

        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        self.tf_broadcaster = StaticTransformBroadcaster()

        self.publish_map_timer = rospy.Timer(rospy.Duration(0.1), self.publish_map)

    def scan_callback(self, scan):
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        for i, r in enumerate(scan.ranges):
            if not math.isinf(r):
                x = r * math.cos(angle_min + angle_increment * i)
                y = r * math.sin(angle_min + angle_increment * i)
                map_x = int((x - self.map_origin_x) / self.map_resolution)
                map_y = int((y - self.map_origin_y) / self.map_resolution)

                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    self.map[map_x, map_y] = 100

    def publish_map(self, event):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = np.ravel(self.map).tolist()

        self.map_publisher.publish(map_msg)

        # Publish static transform between map and odom frame
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "odom"
        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        static_transformStamped.transform.rotation.x = 0
        static_transformStamped.transform.rotation.y = 0
        static_transformStamped.transform.rotation.z = 0
        static_transformStamped.transform.rotation.w = 1

        self.tf_broadcaster.sendTransform(static_transformStamped)

if __name__ == '__main__':
    mapper = LidarOccupancyGridMapper()
    rospy.spin()
