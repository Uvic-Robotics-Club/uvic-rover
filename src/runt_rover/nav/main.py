import dbus
import gps
import rospy
from runt_rover.msg import Coordinates
from runt_rover.nav.report_handlers import TPVReportHandler, DeviceReportHandler
from runt_rover.utils.gps_tpv_mode_enum import TPV_MODE

# Possible TODOs to improve GPS behavior:
# - Implement class as a state machine where each state corresponds to a state of the module/application
# - Ideally: Handle failure such as sudden disconnect from port, leading to failed systemd service.
# - Ideally: Disable appropriate services and perform socket binding such that manual action is not required

GPS_HOST_NAME = 'localhost'
GPS_SOCKET = '2947'
GPS_DEVICE_FILE_NAME = 'ttyUSB0'

class GPSHandler:
    def __init__(self):
        self.pub = rospy.Publisher('gps_coordinates', Coordinates, queue_size=10)
        rospy.init_node('GPS')
        self.rate = rospy.Rate(10) # 10Hz

        # GPS setup
        while True:
            try:
                self.session = gps.gps(GPS_HOST_NAME, GPS_SOCKET)
                break
            except ConnectionRefusedError:
                msg = Coordinates()
                msg.message = 'Connection to GPSD socket refused. Ensure that the socket is enabled and running.'
                
                print(str(msg))
                self.pub.publish(msg)
                self.rate.sleep()
                continue
        
        self.session.stream(gps.WATCH_ENABLE, gps.WATCH_NEWSTYLE)

        # Report handlers
        self.tpv_report_handler = TPVReportHandler()
        self.device_report_handler = DeviceReportHandler()

        self.tpv_report_handler.set_next(self.device_report_handler)
        self.initial_handler = self.tpv_report_handler

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

            try:
                report = self.session.next()

                # Handle and populate message to be published to topic
                ros_msg = self.initial_handler.handle(report)
                if not ros_msg:
                    continue

                print(str(ros_msg))
                self.pub.publish(ros_msg)

            except KeyError:
                pass
            except KeyboardInterrupt:
                quit()
            except StopIteration:
                session = None
                print("GPSD has terminated")
