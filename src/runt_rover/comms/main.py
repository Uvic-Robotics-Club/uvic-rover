import rospy
from runt_rover.comms.network_manager import NetworkManager
from runt_rover.comms.connection_manager import ConnectionManager
import time
import socket

class CommsHandler():
    def __init__(self):
        self.network_manager = NetworkManager()
        self.connection_manager = ConnectionManager()
        self.base_station_wifi_SSID = rospy.get_param("/base_station_wifi_SSID")
        self.base_station_wifi_pwd = rospy.get_param("/base_station_wifi_pwd")

        #self.network_manager.disconnect_from_wifi_network(self.base_station_wifi_SSID)
        #time.sleep(10)
        #self.network_manager.connect_to_wifi_network(self.base_station_wifi_SSID, self.base_station_wifi_pwd)
        #print(self.network_manager.list_available_wifi_networks())
        #print(self.network_manager.list_network_interfaces_status())

        print(self.connection_manager.connect_to_ipv4_host('127.0.0.1', 23103, socket.SOCK_STREAM))