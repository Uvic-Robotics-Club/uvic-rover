import cv2 as cv
from cv_bridge import CvBridge

API_PREFERENCE = cv.CAP_V4L2

class Camera():

    def __init__(self, device_index):
        assert type(device_index) == int

        self.cap = cv.VideoCapture(device_index, API_PREFERENCE)
        self.bridge = CvBridge()

    def get_frame(self):
        ret, frame = self.cap.read()

        if ret:
            return self.bridge.cv2_to_imgmsg(frame)
        return None