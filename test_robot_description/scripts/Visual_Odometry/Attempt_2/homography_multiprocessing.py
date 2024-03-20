import cv2
import numpy as np
import time
from multiprocessing import Process, Manager

url = 'http://192.0.0.4:8080/video'
cap = cv2.VideoCapture(url)

# Set a smaller frame size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

start_time = time.time()
frame_count = 0

prev_frame = None

FLANN_INDEX_LSH = 6
index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)

flann = cv2.FlannBasedMatcher(index_params, search_params)
orb = cv2.ORB_create(nfeatures=10)

cv2.namedWindow('ORB Matches', cv2.WINDOW_NORMAL)

prev_keypoints, prev_descriptors = None, None

def process_frame(frame, frame_count, prev_frame, prev_keypoints, prev_descriptors, result_dict):
    # print('received params')
    frame = cv2.GaussianBlur(frame, (3, 3), 0, 0)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    keypoints = []
    descriptors = []
    grid_size = 8

    rows, cols = gray_frame.shape
    step_size_x = cols // grid_size
    step_size_y = rows // grid_size

    for i in range(grid_size):
        for j in range(grid_size):
            roi = gray_frame[j * step_size_y: (j + 1) * step_size_y, i * step_size_x: (i + 1) * step_size_x]

            kp, desc = orb.detectAndCompute(roi, None)

            for k in kp:
                k.pt = (k.pt[0] + i * step_size_x, k.pt[1] + j * step_size_y)

            if kp and desc is not None:
                keypoints.extend(kp)
                descriptors.extend(desc)

    if prev_descriptors is None:
        prev_descriptors = descriptors

    frame_with_matches = None
    print(len(keypoints), len(descriptors))

    if keypoints and descriptors:
        if prev_descriptors is not None:
            matches = flann.knnMatch(np.asarray(prev_descriptors), np.asarray(descriptors), k=2)

            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.9 * n.distance:
                        good_matches.append(m)

            print(len(prev_keypoints))

            if prev_frame is not None and prev_keypoints is not None:
                if len(good_matches) >= 4:
                    src_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches if m.queryIdx < len(keypoints)])
                    dst_pts = np.float32([prev_keypoints[m.trainIdx].pt for m in good_matches if m.trainIdx < len(prev_keypoints)])

                    if len(src_pts) == len(dst_pts) >= 4:
                        src_pts = src_pts.reshape(-1, 1, 2)
                        dst_pts = dst_pts.reshape(-1, 1, 2)
                        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                        print("Homography Matrix:\n", H)

                        draw_params = dict(singlePointColor=None, matchesMask=None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,)

                        frame_with_matches = cv2.drawMatches(frame, keypoints, prev_frame, prev_keypoints, good_matches, None, **draw_params)

                        frame_count.value += 1
                        elapsed_time = time.time() - start_time
                        fps = frame_count.value / elapsed_time
                        cv2.putText(frame_with_matches, f'FPS: {fps:.2f}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                        prev_keypoints, prev_descriptors = keypoints, descriptors
                        prev_frame = frame.copy()

                    else:
                        prev_keypoints, prev_descriptors = keypoints, descriptors
                        prev_frame = frame.copy()

                else:
                    prev_keypoints, prev_descriptors = keypoints, descriptors
                    prev_frame = frame.copy()

            else:
                prev_keypoints, prev_descriptors = keypoints, descriptors
                prev_frame = frame.copy()

        else:
            prev_keypoints, prev_descriptors = keypoints, descriptors
            prev_frame = frame.copy()

    result_dict['frame_with_matches'] = frame_with_matches

# Use Manager to create a shared dictionary for communication between processes
with Manager() as manager:
    result_dict = manager.dict()

    while True:
        success, frame = cap.read()

        # Create a separate process for frame processing
        process = Process(target=process_frame, args=(frame, frame_count, prev_frame, prev_keypoints, prev_descriptors, result_dict))
        process.start()
        process.join()

        # Get the processed frame from the result_dict
        frame_with_matches = result_dict.get('frame_with_matches')

        # Display the frame with matches
        if frame_with_matches is not None:
            cv2.imshow('ORB Matches', frame_with_matches)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
