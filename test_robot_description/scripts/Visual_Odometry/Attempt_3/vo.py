import cv2
import time

class ORBFeatureDetector:
    def __init__(self):
        self.orb = cv2.ORB_create()
        self.cap = cv2.VideoCapture(0)

    def detect_features(self):
        start_time = time.time()
        num_frames = 0
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)

            # Draw detected keypoints on the frame
            frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)

            # Calculate frame rate
            num_frames += 1
            elapsed_time = time.time() - start_time
            fps = num_frames / elapsed_time

            # Display frame rate on top right corner
            cv2.putText(frame_with_keypoints, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow('ORB Features', frame_with_keypoints)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ORBFeatureDetector()
    detector.detect_features()
