import socket

class ClientConnection():
    pass

class ConnectionManager():
    def __init__(self):
        pass

    def connect_to_ipv4_host(self, host_address, port, socket_type):
        assert(type(host_address)) == str
        assert(type(port)) == int
        assert(type(socket_type)) == socket.SocketKind
        #TODO implement