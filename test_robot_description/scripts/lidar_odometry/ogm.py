#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2

class OccupancyGridMap:
    def __init__(self):
        rospy.init_node('occupancy_grid_map_node', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        self.resolution = 0.05  # Change as needed
        self.map_size_x = 1000   # Change as needed
        self.map_size_y = 1000   # Change as needed
        self.occupancy_threshold = 0.5  # Threshold for marking as occupied

        self.map = np.zeros((self.map_size_x, self.map_size_y), dtype=np.uint8)
        self.map_color = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow('Occupancy Grid Map', cv2.WINDOW_NORMAL)

    def scan_callback(self, data):
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

        # Convert polar coordinates to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Convert Cartesian coordinates to map indices
        map_x = np.round((x - data.range_min) / self.resolution).astype(int)
        map_y = np.round((y - data.range_min) / self.resolution).astype(int)

        # Filter out points outside the map boundaries
        valid_indices = np.logical_and.reduce((
            map_x >= 0, map_x < self.map_size_x,
            map_y >= 0, map_y < self.map_size_y
        ))

        # Mark occupied cells in the map
        self.map[map_x[valid_indices], map_y[valid_indices]] = 255
        self.map_color = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

    def update_plot(self):
        cv2.imshow('Occupancy Grid Map', self.map_color)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        occupancy_grid_map = OccupancyGridMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
