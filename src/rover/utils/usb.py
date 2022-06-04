import re
import subprocess

class USB:
    re_usb_devices = re.compile('ttyUSB\d+')

    @staticmethod
    def list_usb_devices():
        device_files = str(subprocess.check_output(['ls', '/dev/']))
        return USB.re_usb_devices.findall(device_files)

    @staticmethod
    def is_device_connected_at_port(port):
        return port in USB.list_usb_devices()

    @staticmethod
    def bind_device_to_socket(device_filepath, socket_filepath):
        return subprocess.run(['gpsd', device_filepath, '-F', socket_filepath], capture_output=True)
