import serial
import time
import threading

# Replace "COMx" with the actual COM port of your device
ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)

def read_serial():
    while True:
        response = ser.readline().decode().strip()
        if response:
            print(f"Received response: {response}")

# Start the thread for reading serial responses
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True  # Daemonize the thread so it will automatically exit when the main program ends
serial_thread.start()

def send_motor_commands(front_left, front_right, back_left, back_right):
    command = f"{front_left},{front_right},{back_left},{back_right}\n"
    #print(command)
    ser.write(command.encode())

def stop_motors():
    send_motor_commands(0, 0, 0, 0)

def move_forward(speed):
    send_motor_commands(speed, speed, speed, speed)

def move_reverse(speed):
    send_motor_commands(-speed, -speed, -speed, -speed)

def move_left(speed):
    send_motor_commands(-speed, speed, speed, -speed)

def move_right(speed):
    send_motor_commands(speed, -speed, -speed, speed)

def rotate_left(speed):
    send_motor_commands(-speed, speed, -speed, speed)

def rotate_right(speed):
    send_motor_commands(speed, -speed, speed, -speed)

def stop():
    send_motor_commands(0, 0, 0, 0)

try:
    while True:
        speed = 0.1
        #routine = ['w', 's', '4', '6', 'a', 'd', '5']
        #for user_input in routine:
        user_input = input("Enter command (w/s/a/d/4/6/q to quit): ").lower()
        
        if user_input == 'w':
            move_forward(speed)
        elif user_input == 's':
            move_reverse(speed)
        elif user_input == '4':
            move_left(speed)
        elif user_input == '6':
            move_right(speed)
        elif user_input == 'a':
            rotate_left(speed)
        elif user_input == 'd':
            rotate_right(speed)
        elif user_input == '5':
            stop()
        elif user_input == 'q':
            break
        else:
            stop_motors()
        time.sleep(5)
        stop_motors()

except KeyboardInterrupt:
    print("\nScript terminated.")
    stop_motors()
    ser.close()
