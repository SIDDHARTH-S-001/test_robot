import cv2

class ORBFeatureDetector:
    def __init__(self):
        self.orb = cv2.ORB_create()
        self.cap = cv2.VideoCapture(0)

    def detect_features(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)

            # Draw detected keypoints on the frame
            frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=0)

            cv2.imshow('ORB Features', frame_with_keypoints)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ORBFeatureDetector()
    detector.detect_features()
