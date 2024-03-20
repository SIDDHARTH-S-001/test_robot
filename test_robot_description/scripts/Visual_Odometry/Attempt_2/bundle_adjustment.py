import cv2
import numpy as np
import time

# Function to extract keypoints and descriptors from a frame
def extract_features(frame, orb, grid_size):
    keypoints = []
    descriptors = []

    rows, cols = frame.shape
    step_size_x = cols // grid_size
    step_size_y = rows // grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            roi = frame[j * step_size_y: (j + 1) * step_size_y, i * step_size_x: (i + 1) * step_size_x]
            kp, desc = orb.detectAndCompute(roi, None)

            for k in kp:
                k.pt = (k.pt[0] + i * step_size_x, k.pt[1] + j * step_size_y)

            if kp and desc is not None:
                keypoints.extend(kp)
                descriptors.extend(desc)

    return keypoints, np.asarray(descriptors, dtype=np.float32)

url = 'http://192.0.0.4:8080/video'
cap = cv2.VideoCapture(url)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

start_time = time.time()
frame_count = 0

orb = cv2.ORB_create(nfeatures=5)
grid_size = 5

# Previous frame keypoints and descriptors
prev_keypoints, prev_descriptors = None, None

# Create a bundle adjustment object
ba = cv2.detail_BundleAdjusterRay()

while True:
    success, frame = cap.read()
    frame = cv2.GaussianBlur(frame, (3, 3), 0, 0)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    keypoints, descriptors = extract_features(gray_frame, orb, grid_size)

    if prev_keypoints is not None and prev_descriptors is not None:
        # FLANN matching with keypoints and descriptors
        flann = cv2.FlannBasedMatcher(dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1), dict(checks=50))
        
        # Convert descriptors to uint8 type
        descriptors_uint8 = np.uint8(prev_descriptors)
        
        matches = flann.knnMatch(descriptors_uint8, np.uint8(descriptors), k=2)

        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.9 * n.distance:
                    good_matches.append(m)

        src_pts = np.float32([prev_keypoints[m.queryIdx].pt for m in good_matches])
        dst_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches])

        # Create keypoint objects
        src_kp = [cv2.KeyPoint(x=pt[0], y=pt[1]) for pt in src_pts]
        dst_kp = [cv2.KeyPoint(x=pt[0], y=pt[1]) for pt in dst_pts]

        # Convert keypoints to numpy array
        src_kp_np = np.array(src_kp, dtype=np.float32)
        dst_kp_np = np.array(dst_kp, dtype=np.float32)

        # Run bundle adjustment
        status, _, _ = ba.apply(frame, prev_frame, src_kp_np, dst_kp_np, np.eye(3))


        if status:
            print("Bundle Adjustment Successful!")

    prev_frame = frame.copy()
    prev_keypoints, prev_descriptors = keypoints, descriptors

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
