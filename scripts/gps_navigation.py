#!/usr/bin/env python 
import gps
import rospy
from runt_rover.msg import Coordinates
from gps_tpv_mode_enum import TPV_MODE

class GPSHandler:
    def __init__(self):
        self.pub = rospy.Publisher('gps_coordinates', Coordinates, queue_size=10)
        rospy.init_node('GPS', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        # GPS setup
        self.session = gps.gps("localhost", "2947")
        self.session.stream(gps.WATCH_ENABLE, gps.WATCH_NEWSTYLE)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

            try:
                report = self.session.next()
                #print(report)

                # Populate message to be published to topic
                msg = Coordinates()

                if report['class'] != 'TPV':
                    continue

                if hasattr(report, 'mode'):
                    msg.mode = report.mode
                if hasattr(report, 'lat'):
                    msg.latitude = report.lat
                if hasattr(report, 'lon'):
                    msg.longitude = report.lon
                if hasattr(report, 'altMSL'):
                    msg.altitude_msl = report.altMSL

                #print(str(msg))
                self.pub.publish(msg)

            except KeyError:
                pass
            except KeyboardInterrupt:
                quit()
            except StopIteration:
                session = None
                print("GPSD has terminated")

if __name__ == '__main__':
    gps_handler = GPSHandler()
    gps_handler.run()