import subprocess

class Systemd:
    
    @staticmethod
    def stop_service(name):
        return subprocess.run(['systemctl', 'stop', name], capture_output=True)

    @staticmethod
    def disable_service(name):
        return subprocess.run(['systemctl', 'disable', name], capture_output=True)