import cv2
import time

url = 'http://192.0.0.4/8080/video'

cap = cv2.VideoCapture(url)

# Initialize variables for frame rate calculation
start_time = time.time()
frame_count = 0

while True:
    success, frame = cap.read()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB()

    # Divide the image into a grid and ensure uniform feature detection
    grid_size = 8
    rows, cols = frame.shape
    step_size_x = cols // grid_size
    step_size_y = rows // grid_size

    # List to store keypoints
    keypoints = []

    for i in range(grid_size):
        for j in range(grid_size):
            # Define the region of interest (ROI)
            roi = frame[j * step_size_y: (j + 1) * step_size_y, i * step_size_x: (i + 1) * step_size_x]
            
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
    frame_count += 1
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time
    cv2.putText(frame_with_keypoints, f'FPS: {fps:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                cv2.LINE_AA)

    cv2.imshow('Frame', frame_with_keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


