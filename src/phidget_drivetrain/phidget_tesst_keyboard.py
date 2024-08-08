from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
import keyboard
import time

# Motor channel assignments
BACK_LEFT = 0
BACK_RIGHT = 5
FRONT_LEFT = 4
FRONT_RIGHT = 3

# Motor speed (adjust as needed)
SPEED = 0.5

def setup_motor(channel):
    motor = BLDCMotor()
    motor.setChannel(channel)
    motor.openWaitForAttachment(5000)
    return motor

def move_all_motors(motors, velocity):
    for motor in motors:
        motor.setVelocity(velocity)

def main():
    # Initialize motors
    motors = [
        setup_motor(BACK_LEFT),
        setup_motor(BACK_RIGHT),
        setup_motor(FRONT_LEFT),
        setup_motor(FRONT_RIGHT)
    ]

    print("Robot control ready. Use arrow keys to move, 'q' to quit.")

    try:
        while True:
            if keyboard.is_pressed('up'):
                move_all_motors(motors, SPEED)
            elif keyboard.is_pressed('down'):
                move_all_motors(motors, -SPEED)
            else:
                move_all_motors(motors, 0)

            if keyboard.is_pressed('q'):
                break

            time.sleep(0.1)  # Small delay to prevent excessive CPU usage

    finally:
        # Stop all motors and close connections
        move_all_motors(motors, 0)
        for motor in motors:
            motor.close()

if __name__ == "__main__":
    main()