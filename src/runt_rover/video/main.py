import rospy
from runt_rover.video.camera import Camera
from sensor_msgs.msg import Image

class VideoHandler():
    def __init__(self):
        rospy.init_node('video')
        rate = rospy.Rate(20)

        # Initialize video publishers
        self.camera_pub = rospy.Publisher('camera_image', Image, queue_size=10)

        # Initialize cameras
        self.camera = Camera(device_index=0)

        while not rospy.is_shutdown():
            frame = self.camera.get_frame()
            if frame:
                self.camera_pub.publish(self.camera.get_frame())

            # Sleep ROS node
            rate.sleep()

if __name__ == '__main__':
    pass