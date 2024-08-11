import serial
import time

# Open serial connection
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
time.sleep(2)  # Wait for the connection to initialize

# Presets
presets = {
    "reset": ["1 0", "2 0", "3 0", "4 0", "5 0", "6 0"],
    "forward": ["2 100", "3 30", "6 90"],
    "lift": ["2 0", "3 0"]
}

unique_set = {0:0,1:0,2:0,3:0,4:0,5:0}


def send_command(command):
    ser.write(f"{command}\n".encode())
    print(f"Sent: {command}")

def execute_preset(preset_name):
    if preset_name in presets:
        for command in presets[preset_name]:
            send_command(command)
            time.sleep(0.1)  # Small delay between commands
    else:
        print(f"Unknown preset: {preset_name}")

def is_valid_input(input_str):
    print("Current set of states:")
    print(unique_set)

    parts = input_str.split()
    if len(parts) != 2:
        return False
    try:
        motor = int(parts[0])
        angle = float(parts[1])
        unique_set[motor] += angle
        return 1 <= motor <= 6
    except ValueError:
        return False

print("Enter commands (motor angle) or presets (reset, forward, lift). Type 'exit' to quit.")

while True:
    user_input = input("> ").strip().lower()
    
    if user_input == 'exit':
        break
    elif user_input in presets:
        execute_preset(user_input)
    elif is_valid_input(user_input):
        send_command(user_input)
    else:
        print("Invalid input. Please enter 'motor angle' (e.g., '5 45') or a preset name.")

ser.close()
print("Serial connection closed. Exiting.")