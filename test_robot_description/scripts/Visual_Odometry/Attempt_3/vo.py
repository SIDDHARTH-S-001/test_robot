import cv2
import numpy as np
import time
import math
from collections import deque # double ended queue

class ORBFeatureDetector:
    def __init__(self, camera_matrix_file):
        self.orb = cv2.ORB_create(1000) # max orb features detected will be 1000
        self.cap = cv2.VideoCapture(0)
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.camera_matrix = np.loadtxt(camera_matrix_file) # loads camera matrix
        self.distortion_matrix = self.get_distortion_matrix()
        self.transformation_matrices = deque(maxlen=10)  # Buffer to store last 10 transformation matrices
        self.alpha = 0.9  # Smoothing factor for exponential moving average
        self.pose = np.eye(4)
        # verification variables
        self.angle_threshold = 1 * (math.pi / 180)
        self.prev_theta = 0.0
        self.distance_threshold = 0.01 # units: meters
        self.prev_position = None

    def get_distortion_matrix(self):
        k1 = 4.180186921932396715e-02 
        k2 = 2.425581347765151108e+00 
        k3 = 4.395410711636079694e-03 
        p1 = -6.899778925917099577e-03 
        p2 = -2.781548670044756832e+01

        distortion_matrix = np.array([k1, k2, k3, p1, p2])

        return distortion_matrix

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
                    rot = self.pose[:3, :3]
                    trn = self.pose[:3, 3].reshape(3, 1)
                    angle = np.round(math.atan2(rot[1][0], rot[0][0]) * (180 / math.pi), 1)
                    x, y, z = np.round(trn[0][0], 2), np.round(trn[1][0], 2), np.round(trn[2][0], 2)

                    cv2.putText(frame, f'X: {x:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(frame, f'Y: {y:.2f}', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(frame, f'Z: {z:.2f}', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(frame, f'Ang: {angle:.2f}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
                    
                    print("Smoothed Pose:")
                    print(np.round(self.pose, 2))

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
        # Inside the compute_egomotion method
        print("Translation shape:", t.shape)
        print("Translation contents:", t)

        rot, trn = self.check_min_displacement(R, t)
    
        # Compose the rigid body transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = np.round(rot, 2)
        transformation_matrix[:3, 3] = np.round(trn.flatten(), 2)

        return np.round(transformation_matrix, 2)

    def check_min_displacement(self, rotation, translation):
        # Verify Rotation based on constraint
        theta = math.atan2(rotation[1][0], rotation[0][0]) # angle is in radians
        rot = np.eye(3)

        if abs(((theta - self.prev_theta) >= self.angle_threshold)):
            rot = rotation
            self.prev_theta = theta
        else:
            rot = self.yaw_rotation_matrix(self.prev_theta)

        # Verify Translation based on constraint        
        x, y, z = translation[0][0], translation[1][0], translation[2][0]        

        if self.prev_position == None:
            self.prev_position = [0.0, 0.0, 0.0]
            trn = np.array([self.prev_position]).reshape(3, 1)
        else:
            x0, y0, z0 = self.prev_position[0], self.prev_position[1], self.prev_position[2] 
            disp = math.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)
            if disp >= self.distance_threshold:
                self.prev_position[0] = x
                self.prev_position[1] = y
                self.prev_position[2] = z
                trn = translation
            else:
                trn = np.array([self.prev_position]).reshape(3, 1)    

        return np.round(rot, 2), np.round(trn, 2)       

    def yaw_rotation_matrix(self, yaw):
        # Convert yaw angle to radians
        yaw = np.radians(yaw)
        
        # Compute sine and cosine of the yaw angle
        c = np.cos(yaw)
        s = np.sin(yaw)
        
        # Construct the rotation matrix
        Rot = np.array([[c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]])
    
        return Rot

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
    detector = ORBFeatureDetector('camera_matrix.txt')
    detector.detect_features()
