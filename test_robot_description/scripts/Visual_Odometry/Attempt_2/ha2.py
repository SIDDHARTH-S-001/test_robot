import cv2
import numpy as np
import time

url = 'http://192.0.0.4:8080/video'

cap = cv2.VideoCapture(url)

# Set the frame size to a fixed value
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize variables for frame rate calculation
start_time = time.time()
frame_count = 0

# Initialize FLANN parameters
FLANN_INDEX_LSH = 6
index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)  # or pass an empty dictionary

# Create FLANN based matcher
flann = cv2.FlannBasedMatcher(index_params, search_params)

# Initialize ORB with reduced features
orb = cv2.ORB_create(nfeatures=100)  # Adjust the number of features

# Create windows for display
cv2.namedWindow('ORB Features', cv2.WINDOW_NORMAL)
cv2.namedWindow('ORB Matches', cv2.WINDOW_NORMAL)

# Previous frame keypoints and descriptors
prev_keypoints, prev_descriptors = None, None

while True:
    success, frame = cap.read()

    # Remove the resizing operation
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Downscale the frame
    gray_frame = cv2.resize(gray_frame, (480, 320))

    # Detect keypoints and compute descriptors
    keypoints, descriptors = orb.detectAndCompute(gray_frame, None)

    # Check if keypoints and descriptors are not empty
    if keypoints and descriptors is not None:

        # FLANN matching with keypoints and descriptors
        matches = flann.knnMatch(descriptors, descriptors, k=2)

        # Ratio test to filter good matches
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

        # Draw keypoints and matches on the frame
        frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)
        frame_with_matches = cv2.drawMatches(frame, keypoints, frame, keypoints, good_matches, None,
                                             flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        # Calculate and display frame rate
        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        cv2.putText(frame_with_matches, f'FPS: {fps:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)

        # Calculate homography matrix if we have enough good matches and previous keypoints and descriptors
        # Calculate homography matrix if we have enough good matches and previous keypoints and descriptors
        if len(good_matches) >= 4 and prev_keypoints is not None and prev_descriptors is not None:
            # Filter out matches with valid keypoint indices
            src_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches if m.queryIdx < len(keypoints)])
            dst_pts = np.float32([prev_keypoints[m.trainIdx].pt for m in good_matches if m.trainIdx < len(prev_keypoints)])

            if len(src_pts) == len(dst_pts) >= 4:
                # Reshape to the required format
                src_pts = src_pts.reshape(-1, 1, 2)
                dst_pts = dst_pts.reshape(-1, 1, 2)

                # Calculate homography matrix
                H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                # Print the homography matrix
                print("Homography Matrix:\n", H)


        # Update previous keypoints and descriptors for the next iteration
        prev_keypoints, prev_descriptors = keypoints, descriptors

        # Remove the resizing operation
        cv2.imshow('ORB Features', frame_with_keypoints)
        cv2.imshow('ORB Matches', frame_with_matches)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
