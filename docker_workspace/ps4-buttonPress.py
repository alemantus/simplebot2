#!/bin/python

import evdev
from evdev import InputDevice, categorize, ecodes
import subprocess
import time

#subprocess.run(["docker", "container", "rm", "-f", "ros2"])

# Specify the device (event9 in this case)
device_path = '/dev/input/event9'

def execute_command():
    # Run your command here
    # Replace this with the actual command you want to run, for example:
    # subprocess.run(["ls", "-l"])
    print("Button pressed! Running command...")
    subprocess.run(["bash", "/home/alexander/simplebot2/docker_workspace/service_run.sh"])  # Replace with actual command
    time.sleep(1)
    subprocess.run(["systemctl", "stop", "button-listener.service"])
# Set up the input device
device = InputDevice(device_path)

print(f"Listening for events on {device_path}...")

# Listen for events
for event in device.read_loop():
    if event.type == ecodes.EV_KEY:  # Event type 1 (EV_KEY)
        if event.code == ecodes.BTN_SOUTH:  # Code 304 (BTN_SOUTH)
            if event.value == 1:  # Value 1 means the button was pressed
                print("BTN_SOUTH pressed!")
                execute_command()
                time.sleep(1)
                break

