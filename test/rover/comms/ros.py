#!/usr/bin/env python3
PKG='rover'
from re import sub
import roslib; roslib.load_manifest(PKG) # This line is not needed with Catkin
import rospy
import unittest
from unittest.mock import patch

#sys.modules['rospy'] = Mock()
from rover.comms.exceptions import NoConnectionException
from rover.comms.ros import ROS, publishers, subscribers
from rover.msg import Coordinates, Arm, Speed

class TestROS(unittest.TestCase):

    @patch('rover.comms.ros.ConnectionClient')
    @patch('rover.comms.ros.rospy')
    def test_subscribe_gps_coordinates_with_gps_data_success(self, mock_rospy, mock_connection_client):
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
        mock_rospy.loginfo.assert_called_once_with('Received GPS data: {}'.format(expected_telemetry_data))
        mock_connection_client.send_telemetry.assert_called_once_with(expected_telemetry_data)

    @patch('rover.comms.ros.ConnectionClient')
    @patch('rover.comms.ros.rospy')
    def test_subscribe_gps_coordinates_with_gps_data_no_connection_failure(self, mock_rospy, mock_connection_client):
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
        mock_rospy.loginfo.assert_called_with('Received GPS data: {}'.format(expected_telemetry_data))
        mock_connection_client.send_telemetry.assert_called_once_with(expected_telemetry_data)
        #mock_rospy.loginfo.assert_called_with('Unable to send GPS coordinates, no connection.')

    @patch('rover.comms.ros.ConnectionClient')
    def test_subscribe_gps_coordinates_with_invalid_input_type_failure(self, mock_connection_client):
        # Setup
        input_data = object()

        # Act
        self.assertRaises(AssertionError, ROS.subscribe_gps_coordinates, input_data)
        
        # Assert
        mock_connection_client.send_telemetry.assert_not_called()

    def test_assert_expected_ROS_publishers_exist(self):
        # Assert number of publishers
        assert len(publishers) == 2

        # Assert publisher dict keys
        assert 'speed' in publishers
        assert 'arm' in publishers

        # Assert publisher types and parameters
        assert type(publishers['speed']) == rospy.Publisher
        assert type(publishers['arm']) == rospy.Publisher

    def test_assert_expected_ROS_subscribers_exist(self):
        # Assert number of subscribers
        assert len(subscribers) == 2

        # Assert subscriber dict keys
        assert 'gps_coordinates' in subscribers
        assert 'camera_image' in subscribers

        # Assert subscriber types and parameters
        assert type(subscribers['gps_coordinates']) == rospy.Subscriber
        assert type(subscribers['camera_image']) == rospy.Subscriber

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ros', TestROS)