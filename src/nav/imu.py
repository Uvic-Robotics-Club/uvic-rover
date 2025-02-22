#!/usr/bin/env python3
import math
import serial
import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tft

def shutdown_hook(ser_port):
    rospy.loginfo("Shutting down IMU node...")
    ser_port.close()

def main():
    rospy.init_node('imu_talker')
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)    
    imu_msg = Imu()
    rate = rospy.Rate(30)  
    
    ser_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)   # 1/timeout is the frequency at which the port is read
    ser_port.reset_input_buffer() # (serial port may still contain some stray bytes)

    rospy.on_shutdown(lambda: shutdown_hook(ser_port))

    while not rospy.is_shutdown():
        try:
            raw_ser = ser_port.readline().decode().strip()
            if raw_ser:
                parsed_ser = [float(x) for x in raw_ser.split()]
                if len(parsed_ser) == 9:
                    # Compute roll and pitch from accelerometer data:
                    roll = math.atan2(parsed_ser[1], parsed_ser[2])
                    pitch = math.atan2(-parsed_ser[0], math.sqrt(parsed_ser[1]**2 + parsed_ser[2]**2))

                    # Compensate magnetometer readings to compute yaw:
                    mag_x, mag_y, mag_z = parsed_ser[3], parsed_ser[4], parsed_ser[5]
                    mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
                    mag_y_comp = mag_x * math.sin(roll) * math.sin(pitch) + mag_y * math.cos(roll) - mag_z * math.sin(roll) * math.cos(pitch)
                    yaw = math.atan2(-mag_y_comp, mag_x_comp)

                    # Convert to quaternion:
                    quat = tft.quaternion_from_euler(roll, pitch, yaw)

                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.header.frame_id = "imu_frame"
                    imu_msg.orientation.x = quat[0]
                    imu_msg.orientation.y = quat[1]
                    imu_msg.orientation.z = quat[2]
                    imu_msg.orientation.w = quat[3]
                    imu_msg.linear_acceleration.x = parsed_ser[0] #linear acceleration x
                    imu_msg.linear_acceleration.y = parsed_ser[1] #linear acceleration y
                    imu_msg.linear_acceleration.z = parsed_ser[2] #linear acceleration z
                    imu_msg.angular_velocity.x = parsed_ser[6] #angular velocity x
                    imu_msg.angular_velocity.y = parsed_ser[7] #angular velocity y
                    imu_msg.angular_velocity.z = parsed_ser[8] #angular velocity z
                    imu_pub.publish(imu_msg)

                    #Magnetometer paremeters are not included by default in the IMU message format used here
 
                    rospy.loginfo("linear accel - x: %f, y: %f, z: %f", parsed_ser[0], parsed_ser[1], parsed_ser[2])
                    rospy.loginfo("mag - x:%f, y:%f, z:%f", parsed_ser[3], parsed_ser[4], parsed_ser[5])
                    rospy.loginfo("angular vel - x:%f, y:%f, z:%f", parsed_ser[6], parsed_ser[7], parsed_ser[8])
                    rospy.loginfo("orientation - x:%f, y:%f, z:%f", quat[0], quat[1], quat[2])

            rate.sleep()

        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    