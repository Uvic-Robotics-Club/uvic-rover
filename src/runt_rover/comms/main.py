import rospy
from runt_rover.comms.network_manager import NetworkManager

class CommsHandler():
    def __init__(self):
        self.network_manager = NetworkManager()

        # Base station SSID and password fetched from environment variable.
        self.base_station_wifi_SSID = rospy.get_param("/base_station_wifi_SSID")
        self.base_station_wifi_pwd = rospy.get_param("/base_station_wifi_pwd")

        #self.network_manager.disconnect_from_wifi_network(self.base_station_wifi_SSID)
        #time.sleep(10)
        #self.network_manager.connect_to_wifi_network(self.base_station_wifi_SSID, self.base_station_wifi_pwd)
        #print(self.network_manager.list_available_wifi_networks())
        #print(self.network_manager.list_network_interfaces_status())

        #print(self.connection_client.connect_to_http_host('127.0.0.1'))