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
        self.orb = cv2.ORB_create()
        self.flann = cv2.FlannBasedMatcher()

        # Store keypoints and descriptors from the first keyframe
        self.keypoints_first_frame = []
        self.descriptors_first_frame = None

        # Initialize variables for frame rate calculation
        self.start_time = time.time()
        self.frame_count = 0

        # Counter for frame tracking
        self.frame_counter = 0

        # Keyframe saving interval
        self.keyframe_interval = 15

        # Transformation between keyframes
        self.prev_frame = None
        self.prev_keypoints = []
        self.prev_descriptors = None

        # 3D points and camera poses for bundle adjustment
        self.points_3d = []
        self.camera_poses = []

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
                self.prev_frame = gray_frame
                self.prev_keypoints = keypoints
                self.prev_descriptors = desc
                self.points_3d, self.camera_poses = self.initialize_sfm_structure(self.prev_keypoints)

            # Perform FLANN matching with keypoints from the first frame
            matches = self.flann.knnMatch(self.descriptors_first_frame, desc, k=2)

            # Apply ratio test to filter good matches
            good_matches = []
            for m, n in matches:
                if m.distance < 0.8 * n.distance:
                    good_matches.append(m)

            # Draw keypoints and matches on the frame
            frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)

            # Save keyframe every keyframe_interval frames
            if self.frame_counter % self.keyframe_interval == 0:
                # Compute transformation between keyframes only if enough matches are found
                if len(good_matches) >= 4:
                    self.bundle_adjustment()
                    rospy.loginfo('Bundle adjustment performed.')
                else:
                    rospy.loginfo('Not enough matches for bundle adjustment.')

                # Update previous frame and keypoints for the next iteration
                self.prev_frame = gray_frame
                self.prev_keypoints = keypoints
                self.prev_descriptors = desc

            if good_matches:
                frame_with_matches = cv2.drawMatchesKnn(self.keypoints_first_frame, self.descriptors_first_frame,
                                                        gray_frame, keypoints, [good_matches], None,
                                                        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            else:
                frame_with_matches = frame_with_keypoints

            # Calculate and display frame rate
            self.frame_count += 1
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time
            cv2.putText(frame_with_matches, f'FPS: {fps:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)

            # Display the frame with keypoints and matches
            cv2.imshow('ORB Features', frame_with_keypoints)
            cv2.imshow('ORB Matches', frame_with_matches)
            cv2.waitKey(1)

            # Increment frame counter
            self.frame_counter += 1

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def initialize_sfm_structure(self, keypoints):
        # Create 3D points and camera poses for the first keyframe
        points_3d = np.array([keypoint.pt + (0,) for keypoint in keypoints], dtype=np.float32)
        camera_poses = [np.eye(4, dtype=np.float32)]  # Initial camera pose as identity matrix

        return points_3d, camera_poses

    def bundle_adjustment(self):
        # Use OpenCV's SfM module for bundle adjustment
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        _, self.points_3d, self.camera_poses, _, _ = cv2.solvePnPRansac(
            self.points_3d, self.prev_keypoints, np.eye(3), np.zeros(5),
            criteria=criteria, flags=cv2.SOLVEPNP_EPNP
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    feature_detector = FeatureDetectorNode()
    feature_detector.run()
