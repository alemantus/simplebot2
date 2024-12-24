#!/bin/python3
import time
import board
from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from lsm6dsm import IMUNode
from neopixel_indicator import NeopixelNode




def main(args=None):
    rclpy.init(args=args)

    # Initialize IMU node
    i2c = board.I2C()  # or board.SCL and board.SDA if not using STEMMA I2C
    imu_node = IMUNode(i2c)

    # Initialize NeoPixel node
    neopixel_node = NeopixelNode(i2c)
    neopixel_node.blink_pattern(2, 3, 0.2)


    executor = MultiThreadedExecutor()
    executor.add_node(imu_node)
    executor.add_node(neopixel_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
    # Cleanup
        neopixel_node.blink_pattern(2, 2, 0.2)
        neopixel_node.off_pattern()
        imu_node.destroy_node()
        neopixel_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
