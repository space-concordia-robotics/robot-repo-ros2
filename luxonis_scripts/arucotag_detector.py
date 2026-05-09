import cv2

class ArucoTagDetector:
    def __init__(self, dictionary=cv2.aruco.DICT_4X4_250):
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect_tags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        return corners, ids

    def draw_tags(self, image, corners, ids):
        if ids is not None:
            for corner in corners:
                int_corners = corner.astype(int)
                cv2.polylines(image, [int_corners], isClosed=True, color=(0, 255, 0), thickness=2)
        return image