#!/usr/bin/env python

import time
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *
from pynput import keyboard

class MotorController:
    def __init__(self):
        self.motor = BLDCMotor()
        self.motor.setDeviceSerialNumber(751007)  # Replace with your Phidget's serial number
        self.motor.setChannel(0)

        self.motor.openWaitForAttachment(5000)
        
        self.current_velocity = 0
        self.velocity_step = 0.1
        self.max_velocity = 1.0

    def increase_velocity(self):
        self.current_velocity = min(self.current_velocity + self.velocity_step, self.max_velocity)
        self.update_motor()

    def decrease_velocity(self):
        self.current_velocity = max(self.current_velocity - self.velocity_step, -self.max_velocity)
        self.update_motor()

    def stop_motor(self):
        self.current_velocity = 0
        self.update_motor()

    def update_motor(self):
        self.motor.setTargetVelocity(self.current_velocity)
        print(f"Current velocity: {self.current_velocity}")

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.increase_velocity()
            elif key == keyboard.Key.down:
                self.decrease_velocity()
            elif key == keyboard.Key.space:
                self.stop_motor()
            elif key == keyboard.Key.esc:
                return False  # Stop the listener
        except AttributeError:
            pass

    def run(self):
        print("Use arrow keys to control the motor:")
        print("Up arrow: Increase velocity")
        print("Down arrow: Decrease velocity")
        print("Space: Stop motor")
        print("Esc: Exit program")

        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

        self.motor.close()

if __name__ == "__main__":
    controller = MotorController()
    controller.run()