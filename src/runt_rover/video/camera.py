import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

API_PREFERENCE = cv.CAP_V4L2
TEXT_FONT = cv.FONT_HERSHEY_SIMPLEX
TEXT_FONT_SCALE = 1
TEXT_FONT_COLOR = (255, 255, 255)
TEXT_FONT_THICKNESS = 2
TEXT_FONT_LINE_TYPE = 2
TEXT_HORIZONTAL_POS = 10
TEXT_VERTICAL_POS = 470

class Camera():
    '''
    Represents a camera which records video. It is possible to capture
    individual frames from the camera, which is overlayed with a number
    that identifies the frame, showing that the camera is working properly.
    '''

    def __init__(self, device_index):
        '''
        Initializes the camera object with a video capture instance for that 
        specific device.
        '''
        assert type(device_index) == int

        self.frame_id = 0
        self.cap = cv.VideoCapture(device_index, API_PREFERENCE)
        self.bridge = CvBridge()

    def add_text_overlay_to_image(self, frame, text):
        '''
        Overlays text to a cv2 image.
        '''
        assert type(frame) == np.ndarray
        assert type(text) == str
        cv.putText(frame, text, (TEXT_HORIZONTAL_POS, TEXT_VERTICAL_POS), TEXT_FONT, TEXT_FONT_SCALE, TEXT_FONT_COLOR, TEXT_FONT_THICKNESS, TEXT_FONT_LINE_TYPE)

    def get_frame(self):
        '''
        Captures and returns a frame of the camera as a sensor_msgs/Image format,
        which can be used by ROS to send to other nodes as messages. Furthermore,
        a number if overlayes to identify the frame captured and validating that the
        camera is working properly.
        '''
        ret, frame = self.cap.read()
        self.add_text_overlay_to_image(frame, str(self.frame_id))
        self.frame_id+=1
        if self.frame_id > 1000: self.frame_id = 0

        if ret:
            return self.bridge.cv2_to_imgmsg(frame)
        return None