#!/usr/bin/python3
def main():
    print('Hi from motor_controller.')


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import serial
import time
from sensor_msgs.msg import Joy
import math
from std_msgs.msg import Float64MultiArray

class MechDriveNode(Node):
    def __init__(self):
        super().__init__('mech_drive_node')

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

        # Publishers
        self.encoder_pub = self.create_publisher(Float64MultiArray, '/encoder_data', 10)

        self.ser = serial.Serial("/dev/motor_controller", baudrate=115200, timeout=10)
        self.wheel_radius = 0.067/2
        self.wheel_circumference = 2*math.pi*self.wheel_radius


        # Start the thread for reading serial responses
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial(self):
        # Runs in thread
        while True:
            response = self.ser.readline().decode().strip()
            if response:
                try:
                    #self.get_logger().info(f"Received response: {response}")
                    # If the response is a byte string, remove the leading "b" and parse the rest
                    if response.startswith("b'") and response.endswith("'"):
                        response = response[2:-1]
                    encoder_data = [float(val) for val in response.split(' ')]
                    self.publish_encoder_data(encoder_data)
                except ValueError as e:
                    self.get_logger().warn(f"Error parsing encoder data: {e}")
                except Exception as ex:
                    self.get_logger().error(f"Unexpected error: {ex}")

    def publish_encoder_data(self, encoder_data):
        # Convert rotations per second to linear speed in meters per second
        encoder_data_mps = [val * self.wheel_circumference for val in encoder_data]

        msg = Float64MultiArray(data=encoder_data_mps)
        self.encoder_pub.publish(msg)


    def send_motor_commands(self, front_left, front_right, back_left, back_right):
        command = f"{front_left},{front_right},{back_left},{back_right}\n".encode()
        try:
            #self.get_logger().info(f"writing: {command}")
            self.ser.write(command)
        except:
            self.get_logger().info(f"error when writing: {command}")

    def map_value(self, value, in_min=-1, in_max=1, out_min=0, out_max=1):
        # Clamp the input value within the specified range
        value = max(in_min, min(value, in_max))

        # Map the value from the input range to the output range
        mapped_value = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

        return mapped_value

    def velocity_callback(self, msg):
        # Get relevant twist properties
        speed_x = float(msg.linear.x)
        speed_y = float(msg.linear.y)
        rotation_z = float(msg.angular.z)

        # Calculate individual motor speed
        front_left, front_right, back_left, back_right = self.calculate_wheel_speeds(speed_x, speed_y, rotation_z)
        self.send_motor_commands(front_left, front_right, back_left, back_right)

    def rad_per_sec_to_rot_per_sec(self, rad_per_sec):
        rot_per_sec = rad_per_sec / (2 * math.pi)
        return rot_per_sec

    def calculate_wheel_speeds(self, speed_x, speed_y, rotation_z):

        # Car parameters
        wheel_r = 0.07/2

        front_left = self.rad_per_sec_to_rot_per_sec(round(1/wheel_r*(speed_x-speed_y-(0.155+0.18)/2*rotation_z), 2))
        front_right = self.rad_per_sec_to_rot_per_sec(round(1/wheel_r*(speed_x+speed_y+(0.155+0.18)/2*rotation_z), 2))
        back_left = self.rad_per_sec_to_rot_per_sec(round(1/wheel_r*(speed_x+speed_y-(0.155+0.18)/2*rotation_z), 2))
        back_right = self.rad_per_sec_to_rot_per_sec(round(1/wheel_r*(speed_x-speed_y+(0.155+0.18)/2*rotation_z), 2))

        return front_left, front_right, back_left, back_right

    def stop_motors(self):
        self.send_motor_commands(0, 0, 0, 0)

def main(args=None):
    rclpy.init(args=args)

    mech_drive_node = MechDriveNode()

    try:
        rclpy.spin(mech_drive_node)
    except KeyboardInterrupt:
        pass

    mech_drive_node.stop_motors()
    mech_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
	
