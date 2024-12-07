#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
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

        # Declare PID parameters
        self.declare_parameter('VEL_KP', 3.0)
        self.declare_parameter('VEL_KI', 0.0)
        self.declare_parameter('VEL_KD', 0.002)

        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

        # Publishers
        self.encoder_pub = self.create_publisher(Float64MultiArray, '/encoder_data', 10)

        self.ser = serial.Serial("/dev/motor_controller", baudrate=115200, timeout=10)
        self.wheel_radius = 0.067 / 2
        self.wheel_circumference = 2 * math.pi * self.wheel_radius

        # Start the thread for reading serial responses
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def parameter_callback(self, params):
        for param in params:
            if param.name in ['VEL_KP', 'VEL_KI', 'VEL_KD']:
                self.get_logger().info(f"Parameter {param.name} updated to {param.value}")
        

        # Update PID values
        kp = self.get_parameter('VEL_KP').get_parameter_value().double_value
        ki = self.get_parameter('VEL_KI').get_parameter_value().double_value
        kd = self.get_parameter('VEL_KD').get_parameter_value().double_value
        self.update_pid(param.name, param.value)

        return SetParametersResult(successful=True)

    def update_pid(self, name, value):
        params = f"{name}:{value}".encode()
        try:
            self.ser.write(params)
            self.get_logger().info(f"wrote {params}")
        except Exception as e:
            self.get_logger().error(f"Error updating PID: {e}")

    def read_serial(self):
        while True:
            response = self.ser.readline().decode().strip()
            if response:
                try:
                    if response.startswith("b'") and response.endswith("'") and "PID" not in response:
                        response = response[2:-1] # legacy? not used.. 
                    elif "PID" in response:
                        self.get_logger().info(f"PID set response: {response}")
                    try:
                        encoder_data = [float(val) for val in response.split(' ')]
                        self.publish_encoder_data(encoder_data)
                    except:
                        self.get_logger().warning(f"Missed and encoder pub")
                except ValueError as e:
                    self.get_logger().warn(f"Error parsing encoder data: {e}")
                except Exception as ex:
                    self.get_logger().error(f"Unexpected error: {ex}")

    def publish_encoder_data(self, encoder_data):
        encoder_data_mps = [val * self.wheel_circumference for val in encoder_data]
        msg = Float64MultiArray(data=encoder_data_mps)
        self.encoder_pub.publish(msg)

    def send_motor_commands(self, front_left, front_right, back_left, back_right):
        command = f"{front_left},{front_right},{back_left},{back_right}\n".encode()
        try:
            self.ser.write(command)
        except:
            self.get_logger().info(f"Error writing command: {command}")

    def velocity_callback(self, msg):
        speed_x = float(msg.linear.x)
        speed_y = float(msg.linear.y)
        rotation_z = float(msg.angular.z)
        front_left, front_right, back_left, back_right = self.calculate_wheel_speeds(speed_x, speed_y, rotation_z)
        self.send_motor_commands(front_left, front_right, back_left, back_right)

    def rad_per_sec_to_rot_per_sec(self, rad_per_sec):
        return rad_per_sec / (2 * math.pi)

    def calculate_wheel_speeds(self, speed_x, speed_y, rotation_z):
        wheel_r = 0.07 / 2
        front_left = self.rad_per_sec_to_rot_per_sec(round(1 / wheel_r * (speed_x - speed_y - (0.155 + 0.18) / 2 * rotation_z), 2))
        front_right = self.rad_per_sec_to_rot_per_sec(round(1 / wheel_r * (speed_x + speed_y + (0.155 + 0.18) / 2 * rotation_z), 2))
        back_left = self.rad_per_sec_to_rot_per_sec(round(1 / wheel_r * (speed_x + speed_y - (0.155 + 0.18) / 2 * rotation_z), 2))
        back_right = self.rad_per_sec_to_rot_per_sec(round(1 / wheel_r * (speed_x - speed_y + (0.155 + 0.18) / 2 * rotation_z), 2))
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
