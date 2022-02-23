import rospy
from runt_rover.comms.connection_server import *
from runt_rover.comms.wifi import WifiManager


class CommsHandler():
    def __init__(self):
        rospy.init_node('comms')

        self.wifi_manager = WifiManager()

        # Base station SSID and password fetched from environment variable.
        self.base_station_wifi_SSID = rospy.get_param("/base_station_wifi_SSID")
        self.base_station_wifi_pwd = rospy.get_param("/base_station_wifi_pwd")
        self.comms_server_port = rospy.get_param("/comms_server_port")

        # Connect to WiFi network.
        # TODO: REMOVE WHEN WORKING WITHIN VM.
        #if not self.wifi_manager.is_connected_to_wifi():
        #    self.wifi_manager.connect_to_wifi_network(self.base_station_wifi_SSID, self.base_station_wifi_pwd)
        #assert self.wifi_manager.is_connected_to_wifi()

        # Start server
        app.run(host='0.0.0.0', port=self.comms_server_port)