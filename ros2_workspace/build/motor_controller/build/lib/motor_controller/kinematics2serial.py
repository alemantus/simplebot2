def main():
    print('Hi from motor_controller.')


if __name__ == '__main__':
    main()
#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import serial
import time

class MechDriveNode(Node):
    def __init__(self):
        super().__init__('mech_drive_node')

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Replace "COMx" with the actual COM port of your device
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=10)
        time.sleep(1)
        # Start the thread for reading serial responses
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial(self):
        while True:
            #response = 1
            response = self.ser.readline().decode().strip()
            if response:
                try:
                    self.get_logger().info(f"Received response: {response}")
                except:
                    self.get_logger().info(f"read")
            time.sleep(1)

    def send_motor_commands(self, front_left, front_right, back_left, back_right):
        command = f"{front_left},{front_right},{back_left},{back_right}\n".encode()
        try:
            self.get_logger().info(f"writing: {command}")
            self.ser.write(command)
        except:
            self.get_logger().info(f"error when writing: {command}")


    def velocity_callback(self, msg):
        #self.get_logger().info(msg)
        speed_x = msg.linear.x
        speed_y = msg.linear.y
        rotation_z = msg.angular.z

        # Assuming you have a function to calculate individual wheel speeds based on the desired velocity and rotation
        front_left, front_right, back_left, back_right = self.calculate_wheel_speeds(speed_x, speed_y, rotation_z)
        self.get_logger().info(f"fl, fr, bl, br: {front_left}, {front_right}, {back_left}, {back_right}")
        self.send_motor_commands(front_left, front_right, back_left, back_right)

    def rad_per_sec_to_meters_per_sec(self, rad_per_sec):
        radius = 0.07/2
        linear_velocity = radius * rad_per_sec
        return linear_velocity

    def calculate_wheel_speeds(self, speed_x, speed_y, rotation_z):
        # Implement your kinematics calculation here
        # This is a placeholder and needs to be replaced with actual kinematics calculation
        # You might need to consider the robot's dimensions, wheel positions, and wheel angles
        wheel_r = 0.07/2

        lx_l = -0.09
        lx_r = 0.09

        ly_front = 0.065
        ly_back = 0.065


        front_left = self.rad_per_sec_to_meters_per_sec(round(1/wheel_r*(speed_x-speed_y-(0.155+0.18)*rotation_z), 2))
        front_right = self.rad_per_sec_to_meters_per_sec(round(1/wheel_r*(speed_x+speed_y+(0.155+0.18)*rotation_z), 2))
        back_left = self.rad_per_sec_to_meters_per_sec(round(1/wheel_r*(speed_x+speed_y-(0.155+0.18)*rotation_z), 2))
        back_right = self.rad_per_sec_to_meters_per_sec(round(1/wheel_r*(speed_x-speed_y+(0.155+0.18)*rotation_z), 2))
        

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
	