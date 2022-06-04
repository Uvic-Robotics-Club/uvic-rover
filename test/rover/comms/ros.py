#!/usr/bin/env python3
PKG='rover'
from unittest import mock
import roslib; roslib.load_manifest(PKG) # This line is not needed with Catkin
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.modules['rospy'] = MagicMock()
from rover.comms.exceptions import NoConnectionException
from rover.comms.ros import ROS
from rover.msg import Coordinates

class TestROS(unittest.TestCase):

    @patch('rover.comms.ros.ConnectionClient')
    def test_subscribe_gps_coordinates_with_gps_data_success(self, mock_connection_client):
        # Setup
        input_data = Coordinates()
        input_data.mode = 1
        input_data.latitude = 45.34324
        input_data.longitude = -132.01231
        input_data.altitude_msl = 54.5
        input_data.message = None

        expected_telemetry_data = {
            'GPS': {
                'latitude': input_data.latitude,
                'longitude': input_data.longitude,
                'altitude_msl': input_data.altitude_msl,
                'mode': input_data.mode,
                'message': input_data.message
            }
        }

        # Act
        ROS.subscribe_gps_coordinates(input_data)

        # Assert
        mock_connection_client.send_telemetry.assert_called_once_with(expected_telemetry_data)

    @patch('rover.comms.ros.ConnectionClient')
    def test_subscribe_gps_coordinates_with_gps_data_no_connection_failure(self, mock_connection_client):
        # Setup
        input_data = Coordinates()
        input_data.mode = 1
        input_data.latitude = 45.34324
        input_data.longitude = -132.01231
        input_data.altitude_msl = 54.5
        input_data.message = None

        expected_telemetry_data = {
            'GPS': {
                'latitude': input_data.latitude,
                'longitude': input_data.longitude,
                'altitude_msl': input_data.altitude_msl,
                'mode': input_data.mode,
                'message': input_data.message
            }
        }

        mock_connection_client.subscribe_gps_coordinates.side_effect = NoConnectionException()

        # Act
        ROS.subscribe_gps_coordinates(input_data)

        # Assert
        mock_connection_client.send_telemetry.assert_called_once_with(expected_telemetry_data)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ros', TestROS)