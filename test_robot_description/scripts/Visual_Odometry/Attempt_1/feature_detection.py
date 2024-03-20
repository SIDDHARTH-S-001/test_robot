#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time



class FeatureDetectorNode:
    def __init__(self):
        rospy.init_node('feature_detector_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Initialize variables for frame rate calculation
        self.start_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ORB features
            orb = cv2.ORB_create(1000)
            
            # Divide the image into a grid and ensure uniform feature detection
            grid_size = 8
            rows, cols = gray_frame.shape
            step_size_x = cols // grid_size
            step_size_y = rows // grid_size

            # List to store keypoints
            keypoints = []

            for i in range(grid_size):
                for j in range(grid_size):
                    # Define the region of interest (ROI)
                    roi = gray_frame[j * step_size_y: (j + 1) * step_size_y, i * step_size_x: (i + 1) * step_size_x]
                    
                    # Detect keypoints in the ROI
                    kp, _ = orb.detectAndCompute(roi, None)
                    
                    # Offset keypoints' coordinates based on the grid
                    for k in kp:
                        k.pt = (k.pt[0] + i * step_size_x, k.pt[1] + j * step_size_y)
                    
                    # Add keypoints to the list
                    keypoints.extend(kp)

            # Draw keypoints on the frame
            frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)

            # Calculate and display frame rate
            self.frame_count += 1
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time
            cv2.putText(frame_with_keypoints, f'FPS: {fps:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)

            # Display the frame with keypoints
            cv2.imshow('ORB Features', frame_with_keypoints)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    feature_detector = FeatureDetectorNode()
    feature_detector.run()
