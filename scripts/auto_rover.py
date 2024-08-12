#!/usr/bin/env python

# -----------------------------
#   USAGE
# -----------------------------
# python detect_aruco_image.py --image images/example_01.png --type DICT_5X5_100

# -----------------------------
#   IMPORTS
# -----------------------------
# Import the necessary packages

# this script will be running on the rover

import imutils
import cv2
import time
import rospy
from Phidget22.Phidget import *
from Phidget22.Devices.BLDCMotor import *




def initialize_motor(serial_number, hub_port):
    motor = BLDCMotor()
    motor.setDeviceSerialNumber(serial_number)
    motor.setHubPort(hub_port)
    motor.setIsHubPortDevice(False)
    motor.openWaitForAttachment(5000)
    return motor

vint_hub_serial_number = 757580

motors = {
    'back_left': initialize_motor(vint_hub_serial_number, 5),
    'back_right': initialize_motor(vint_hub_serial_number, 0),
    'front_left': initialize_motor(vint_hub_serial_number, 4),
    'front_right': initialize_motor(vint_hub_serial_number, 3)
}




### ARUCO STUFF HERE DOWN

# Define the names of each possible ArUco tag that OpenCV supports
ARUCO = {"DICT": cv2.aruco.DICT_4X4_100}
IMAGE = {"PATH": "alice.jpg"}
# Load the input image from disk and resize it
#print("[INFO] Loading image...")
print("booting up auto aruco")


motors['back_left'].setTargetVelocity(0.01)
motors['back_right'].setTargetVelocity(0.01)
motors['front_left'].setTargetVelocity(0.01)
motors['front_right'].setTargetVelocity(0.01)

while (True):
    image = cv2.imread(IMAGE["PATH"])
    #image = imutils.resize(image, width=600)

    # Load the ArUCo dictionary, grab the ArUCo parameters and detect the markers
    #print("[INFO] Detecting '{}' tags...".format(ARUCO["DICT"]))
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO["DICT"])
    arucoParams = cv2.aruco.DetectorParameters()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    # print(ids)

    # Verify *at least* one ArUCo marker was detected
    if len(corners) > 0:
        motors['back_left'].setTargetVelocity(0)
        motors['back_right'].setTargetVelocity(0)
        motors['front_left'].setTargetVelocity(0)
        motors['front_right'].setTargetVelocity(0)
        # Flatten the ArUCo IDs list
        ids = ids.flatten()
        # Loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # Extract the markers corners which are always returned in the following order:
            # TOP-LEFT, TOP-RIGHT, BOTTOM-RIGHT, BOTTOM-LEFT
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # Convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # Draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            # Compute and draw the center (x, y) coordinates of the ArUCo marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # Draw the ArUco marker ID on the image
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            # Show the output image
        # stop rover

