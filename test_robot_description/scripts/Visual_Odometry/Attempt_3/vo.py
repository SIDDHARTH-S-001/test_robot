import cv2
import time

class ORBFeatureDetector:
    def __init__(self):
        self.orb = cv2.ORB_create(1000)
        self.cap = cv2.VideoCapture(0)
        self.prev_keypoints = None
        self.prev_descriptors = None

    def detect_features(self):
        start_time = time.time()
        num_frames = 0
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)

            # Match features with previous frame
            if self.prev_keypoints is not None and self.prev_descriptors is not None and keypoints is not None and descriptors is not None:
                matches = self.match_features(descriptors)
                if matches:
                    # Draw matches
                    frame_with_matches = cv2.drawMatches(self.prev_frame, self.prev_keypoints, frame, keypoints, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                    cv2.imshow('ORB Features with Matches', frame_with_matches)

            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.prev_frame = frame.copy()

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

        # Apply ratio test
        good_matches = []
        if matches is not None:
            if len(matches) > 0:  # Check if there are any matches
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)

        return good_matches

if __name__ == "__main__":
    detector = ORBFeatureDetector()
    detector.detect_features()
