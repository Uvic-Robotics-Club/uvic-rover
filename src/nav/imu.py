#!/usr/bin/env python3

import serial
import rospy
from sensor_msgs.msg import Imu

def get_parsed_serial_data(raw_serial_data):
    listed_ser_data = raw_serial_data.split()
    parsed_ser_data = []
    for x in listed_ser_data:
        parsed_ser_data.append(float(x))  
    return parsed_ser_data 

def shutdown_hook(ser_port):
    rospy.loginfo("Shutting down IMU node...")
    ser_port.close()

def main():
    rospy.init_node('imu_talker')
    imu_pub = rospy.Publisher('IMU', Imu, queue_size=10)    
    imu_msg = Imu()
    #rate = rospy.Rate(0.1)  

    ser_port = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)   # 1/timeout is the frequency at which the port is read
    rospy.on_shutdown(lambda: shutdown_hook(ser_port))

    while not rospy.is_shutdown():
        try:
            raw_serial_data = ser_port.readline().decode().strip()
            if raw_serial_data:
                imu_data = get_parsed_serial_data(raw_serial_data)
                if len(imu_data) == 9:
                    imu_msg.linear_acceleration.x = imu_data[0]
                    imu_msg.linear_acceleration.y = imu_data[1]
                    imu_msg.linear_acceleration.z = imu_data[2]
                    imu_msg.orientation.x = imu_data[6]
                    imu_msg.orientation.y = imu_data[7]
                    imu_msg.orientation.z = imu_data[8]
                    imu_pub.publish(imu_msg)
        
                    print("Accel:", imu_data[0], imu_data[1], imu_data[2])
                    print("Mag:", imu_data[3], imu_data[4], imu_data[5])
                    print("Gyro:", imu_data[6], imu_data[7], imu_data[8])

            #rate.sleep()

        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    