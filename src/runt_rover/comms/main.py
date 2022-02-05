import rospy
from runt_rover.comms.network_manager import NetworkManager

class CommsHandler():
    def __init__(self):
        self.network_manager = NetworkManager()
        self.base_station_wifi_SSID = rospy.get_param("/base_station_wifi_SSID")
        self.base_station_wifi_pwd = rospy.get_param("/base_station_wifi_pwd")

        self.network_manager.connect_to_wifi_network(self.base_station_wifi_SSID, self.base_station_wifi_pwd)
        #self.network_manager.disconnect_from_wifi_network(self.base_station_wifi_SSID)