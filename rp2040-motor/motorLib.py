import select
import sys
import time
from motor import Motor, motor2040
from encoder import Encoder, MMME_CPR
from pimoroni import Button, PID, REVERSED_DIR
import gc
import math 

# Wheel friendly names
C = 2
D = 3
B = 1
A = 0

GEAR_RATIO = 21.3                         # The gear ratio of the motors
COUNTS_PER_REV = 44 * GEAR_RATIO    # The counts per revolution of each motor's output shaft

SPEED_SCALE = 3.78                        # The scaling to apply to each motor's speed to match its real-world speed

UPDATES = 100                             # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
ZERO_POINT = 0.0                      # The duty cycle that corresponds with zero speed when plotting the motor's speed as a straight line
DEAD_ZONE = 0.0                        # The duty cycle below which the motor's friction prevents it from moving


# PID values
VEL_KP = 20.0                             # Velocity proportional (P) gain
VEL_KI = 0.005                            # Velocity integral (I) gain
VEL_KD = 0.4                              # Velocity derivative (D) gain

UPDATE_RATE_ACCUM = 0
# Free up hardware resources ahead of creating a new Encoder
gc.collect()

# Create a list of motors with a given speed scale
MOTOR_PINS = [motor2040.MOTOR_A, motor2040.MOTOR_B, motor2040.MOTOR_C, motor2040.MOTOR_D]
motors = [Motor(pins, speed_scale=SPEED_SCALE, zeropoint=ZERO_POINT, deadzone=DEAD_ZONE) for pins in MOTOR_PINS]

# Create a list of encoders, using PIO 0, with the given counts per revolution
ENCODER_PINS = [motor2040.ENCODER_A, motor2040.ENCODER_B, motor2040.ENCODER_C, motor2040.ENCODER_D]
ENCODER_NAMES = ["C", "D", "B", "A"]
encoders = [Encoder(0, i, ENCODER_PINS[i], counts_per_rev=COUNTS_PER_REV, count_microsteps=True) for i in range(motor2040.NUM_MOTORS)]

CAPTURE_TIME = 0.2  
# Reverse the direction of the B and D motors and encoders
#motors[FL].direction(REVERSED_DIR)
#motors[RL].direction(REVERSED_DIR)
encoders[B].direction(REVERSED_DIR)
encoders[D].direction(REVERSED_DIR)

# Create the user button
user_sw = Button(motor2040.USER_SW)

# Create PID objects for position control
vel_pids = [PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE) for i in range(motor2040.NUM_MOTORS)]


class MySerial:
    def __init__(self):
        # Set up the poll object
        self.poll_obj = select.poll()
        self.poll_obj.register(sys.stdin, select.POLLIN)
        self.data = ""

    def read_serial(self):
        self.poll_results = self.poll_obj.poll(1)
        if self.poll_results:
            self.data = sys.stdin.readline().strip()
            print(self.data)
            return self.data
        return None

    def send_serial(self, data):
        sys.stdout.write((f"{data}\n").encode())
        return 1


def set_motor_speeds(motors, speeds):
    for i in range(min(len(speeds), motor2040.NUM_MOTORS)):
        vel_pids[i].setpoint = speeds[i]

def rpm_to_mps(rpm, radius=0.07/2):
    linear_speed = (rpm * 2 * math.pi * radius) / 60
    return linear_speed

def mps_to_rpm(linear_speed):
    radius=0.067/2
    rpm = linear_speed/(2*math.pi*radius)
    # rpm = (linear_speed * 60) / (2 * math.pi * radius)
    return rpm

# Create instances
serial_obj = MySerial()

# Enable the motor to get started
for m in motors:
    m.enable()

captures = [None] * motor2040.NUM_MOTORS

serial_start = None
# Continually move the motor until the user button is pressed
while not user_sw.raw():
    # Capture the state of all the encoders
    for i in range(motor2040.NUM_MOTORS):
        captures[i] = encoders[i].capture()
    #print(f"rpm motor: {captures[0].revolutions_per_second}")

    #print("sup")
    for i in range(motor2040.NUM_MOTORS):
        # Calculate the acceleration to apply to the motor to move it closer to the velocity setpoint
        accel = vel_pids[i].calculate(captures[i].revolutions_per_second)

        # Accelerate or decelerate the motor
        motors[i].speed(motors[i].speed() + (accel * UPDATE_RATE))
    try:
        # Your MicroPython code that may raise an exception
        # ...

        # Read serial input for controlling motors
        serial_input = serial_obj.read_serial()
        
        if serial_input is not None:
            speeds = []
            try:
                for speed in serial_input.split(","):
                    #speed = mps_to_rpm(float(speed))
                    speeds.append(float(speed))
                set_motor_speeds(motors, speeds)
                serial_start = 1
            except:
                serial_obj.send_serial("error1")
    except Exception as e:
        # Print the last exception
        serial_obj.send_serial(e)
    # Send RPM values back over serial
    # rpm_values = [captures[i].revolutions_per_second * 60 for i in range(motor2040.NUM_MOTORS)]
    # serial_obj.send_serial(",".join(map(str, rpm_values)))
    #enc0 = encoders[0].capture()


    
    time.sleep(UPDATE_RATE)
    #enc0  = encoders[0].capture()
    UPDATE_RATE_ACCUM += 0.1
    if UPDATE_RATE_ACCUM >= 0.50:
        serial_obj.send_serial((f"rpm motor measured:target -").encode())
        UPDATE_RATE_ACCUM = 0
# Stop all the motors
for m in motors:
    m.disable()