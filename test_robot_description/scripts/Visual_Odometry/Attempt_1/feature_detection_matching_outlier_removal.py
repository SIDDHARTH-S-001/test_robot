#!/usr/bin/env python3

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
        self.orb = cv2.ORB_create(1000)
        self.flann = cv2.FlannBasedMatcher()

        # Store keypoints from the first frame
        self.keypoints_first_frame = []
        self.descriptors_first_frame = None

        # Initialize variables for frame rate calculation
        self.start_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
                    kp, desc = self.orb.detectAndCompute(roi, None)

                    # Offset keypoints' coordinates based on the grid
                    for k in kp:
                        k.pt = (k.pt[0] + i * step_size_x, k.pt[1] + j * step_size_y)

                    # Add keypoints to the list
                    keypoints.extend(kp)

            # If this is the first frame, store keypoints and descriptors
            if not self.keypoints_first_frame:
                self.keypoints_first_frame = keypoints
                self.descriptors_first_frame = desc

            # Perform FLANN matching with keypoints from the first frame
            matches = self.flann.knnMatch(self.descriptors_first_frame, desc, k=2)

            # Apply ratio test to filter good matches
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

            # RANSAC parameters
            ransac_threshold = 3.0

            # Extract coordinates of good matches
            src_pts = np.float32([self.keypoints_first_frame[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Use 5-point RANSAC to remove outliers
            _, mask = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_RANSAC, ransac_threshold)

            # Check if there are enough inlier points for fundamental matrix calculation
            if mask is not None:
                inliers = mask.ravel().astype(bool)

                # Apply the mask to keep only inliers
                good_matches_ransac = [good_matches[i] for i in range(len(good_matches)) if inliers[i]]

                # Draw keypoints and matches on the frame
                frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
                frame_with_matches_ransac = cv2.drawMatchesKnn(self.keypoints_first_frame, self.descriptors_first_frame,
                                                            gray_frame, keypoints, [good_matches_ransac], None,
                                                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # Calculate and display frame rate
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time
                cv2.putText(frame_with_matches_ransac, f'FPS: {fps:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1, cv2.LINE_AA)

                # Display the frames
                cv2.imshow('Detected Features', frame_with_keypoints)
                cv2.imshow('Matches with RANSAC', frame_with_matches_ransac)
                cv2.waitKey(1)
            else:
                # Handle the case when there are not enough inliers
                rospy.loginfo("Not enough inliers for RANSAC.")

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    feature_detector = FeatureDetectorNode()
    feature_detector.run()
