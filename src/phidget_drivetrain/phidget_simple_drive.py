import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
from sensor_msgs.msg import Joy

# Return BLDC motor phidget object
def initialize_motor(serial_number, hub_port):
    motor = BLDCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setHubPort(hub_port)
    motor.setIsHubPortDevice(False)
    motor.openWaitForAttachment(5000)
    return motor

# Callback function for joy messages
def joy_callback(data):
    # Use the vertical axis of the left stick for forward/backward control
    forward = data.axes[4]  # Assuming left stick vertical axis is at index 1

    # Set all motors to the same velocity for forward/backward movement
    motors['back_left'].setTargetVelocity(forward)
    motors['back_right'].setTargetVelocity(forward)
    motors['front_left'].setTargetVelocity(forward)
    motors['front_right'].setTargetVelocity(forward)

def main():
    global motors

    # Define the serial number of your VINT Hub
    vint_hub_serial_number = 751007  # Replace with your VINT Hub's serial number

    # Initialize motors
    motors = {
        'back_left': initialize_motor(vint_hub_serial_number, 0),
        'back_right': initialize_motor(vint_hub_serial_number, 5),
        'front_left': initialize_motor(vint_hub_serial_number, 4),
        'front_right': initialize_motor(vint_hub_serial_number, 3)
    }

    # Initialize ROS node and subscribe to joy topic
    rospy.init_node('phidget_motor_control')
    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.spin()

    # Close motors when done
    for motor in motors.values():
        motor.close()

if __name__ == '__main__':
    main()
