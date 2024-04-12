import cv2
import numpy as np
import time
from collections import deque # double ended queue

class ORBFeatureDetector:
    def __init__(self, camera_matrix_file, distortion_matrix_file):
        self.orb = cv2.ORB_create(1000) # max orb features detected will be 1000
        self.cap = cv2.VideoCapture(0)
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.camera_matrix = np.loadtxt(camera_matrix_file) # loads camera matrix
        self.distortion_matrix = np.loadtxt(distortion_matrix_file)
        self.transformation_matrices = deque(maxlen=10)  # Buffer to store last 10 transformation matrices
        self.alpha = 0.9  # Smoothing factor for exponential moving average
        self.pose = np.eye(4)

    def detect_features(self):
        start_time = time.time()
        num_frames = 0
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Remove distortions before detecting features
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.distortion_matrix)
            # Converting the frame to grayscale before detecting features
            gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY) 
            # Detecting features and getting keypoints and descriptions
            keypoints, descriptors = self.orb.detectAndCompute(gray, None) 

            # Match features with previous frame
            if self.prev_keypoints is not None and self.prev_descriptors is not None and keypoints is not None and descriptors is not None:
                matches = self.match_features(descriptors)
                if matches:
                    # Compute essential matrix and egomotion
                    T = self.compute_egomotion(keypoints, matches)
                    # Add new transformation to buffer
                    self.transformation_matrices.append(T)  
                    # Apply smoothing
                    smoothed_transform = self.smooth_transform()  
                    # Final pose
                    self.pose *= smoothed_transform
                    print("Smoothed Pose:")
                    print(self.pose)

            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors

            # Calculate frame rate
            num_frames += 1
            elapsed_time = time.time() - start_time
            fps = num_frames / elapsed_time

            # Display frame rate on top right corner
            cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow('ORB Features', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def match_features(self, descriptors):
        # FLANN parameters
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)  # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.prev_descriptors, descriptors, k=2)

        # Apply Lowe's ratio test (https://stackoverflow.com/questions/51197091/how-does-the-lowes-ratio-test-work)
        good_matches = []
        if matches is not None:
            if len(matches) > 0:  # Check if there are any matches
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.7 * n.distance:
                            good_matches.append(m)
        return good_matches

    def compute_egomotion(self, keypoints, matches):
        points1 = np.float32([self.prev_keypoints[m.queryIdx].pt for m in matches])
        points2 = np.float32([keypoints[m.trainIdx].pt for m in matches])

        # Reshape points1 and points2 if necessary
        if not points1.flags['C_CONTIGUOUS']:
            points1 = np.ascontiguousarray(points1)
        if not points2.flags['C_CONTIGUOUS']:
            points2 = np.ascontiguousarray(points2)

        points1 = points1.reshape(-1, 1, 2)
        points2 = points2.reshape(-1, 1, 2)

        # Compute essential matrix
        E, mask = cv2.findEssentialMat(points1, points2, self.camera_matrix, cv2.RANSAC, 0.999, 2.0, None)

        # Recover pose from essential matrix
        _, R, t, _ = cv2.recoverPose(points1=points1, points2=points2, E=E, cameraMatrix=self.camera_matrix)

        # Compose the rigid body transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = np.round(R, 2)
        transformation_matrix[:3, 3] = np.round(t.flatten(), 2)

        return np.round(transformation_matrix, 2)

    def smooth_transform(self):
        if len(self.transformation_matrices) == 0:
            # Return identity matrix if no transformations in buffer
            return np.eye(4)  

        # Apply moving average to smooth the transformation matrices
        smoothed_transform = self.transformation_matrices[0]  # Initialize with the first transformation
        for i in range(1, len(self.transformation_matrices)):
            smoothed_transform = self.alpha * self.transformation_matrices[i] + (1 - self.alpha) * smoothed_transform
        return np.round(smoothed_transform, 2)
    
if __name__ == "__main__":
    detector = ORBFeatureDetector('camera_matrix.txt', 'distortion_matrix.txt')
    detector.detect_features()
