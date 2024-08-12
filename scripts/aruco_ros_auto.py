#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed2i/zed_node/rgb_raw/image_raw_color", Image, self.image_callback)
        self.save_path = "/home/summit-uvic/catkin_ws/src/uvic_rover/scripts"  # Replace with your desired folder path
        self.last_saved_time = time.time()

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Save image every 5 seconds
        current_time = time.time()
        if current_time - self.last_saved_time >= 1.0:
            timestamp = rospy.Time.now()
            image_name = os.path.join(self.save_path, f"alice.jpg")
            cv2.imwrite(image_name, cv_image)
            rospy.loginfo(f"Saved image {image_name}")
            self.last_saved_time = current_time

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    rospy.spin()