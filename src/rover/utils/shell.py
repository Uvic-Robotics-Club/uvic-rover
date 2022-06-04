import subprocess
import os

sudo_password = None
SUDO_PASSWORD_FILE = '/sudo_password.txt'
with open(SUDO_PASSWORD_FILE, 'r') as f:
    sudo_password = f.readline()

class Shell:
    @staticmethod
    def execute_sudo_command(command):
        assert type(command) == str
        command_list = command.split()
        echo_password_cmd = subprocess.Popen(['echo', sudo_password], stdout=subprocess.PIPE)
        return subprocess.Popen(['sudo', '-S'] + command_list, stdin=echo_password_cmd.stdout, stdout=subprocess.PIPE)
