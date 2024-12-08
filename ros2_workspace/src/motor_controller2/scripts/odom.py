#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 50)
        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'encoder_data',
            self.encoder_callback,
            10)
        self.subscription_

        self.current_time_ = self.get_clock().now().to_msg()
        self.last_time_ = self.get_clock().now().to_msg()
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.vx_ = 0.0
        self.vy_ = 0.0
        self.vth_ = 0.0
        self.publish_odometry()

        self.timer_ = self.create_timer(0.1, self.publish_odometry)  # 0.1 seconds = 10 Hz

        
        self.wheel_separation_ = 0.222  # Distance between wheels (meters)
        self.wheel_separation_length_ = 0.13  # Distance between front and rear wheels (meters)

    def encoder_callback(self, msg):
        # Assuming the encoder message has 4 elements: [front_left, front_right, rear_left, rear_right]
        front_left, front_right, rear_left, rear_right = msg.data

        # Update linear velocities from encoder readings
        self.vx_ = (front_left + front_right + rear_left + rear_right) * 0.25  # Forward/backward
        self.vy_ = (-front_left + front_right + rear_left - rear_right) * 0.25  # Left/right
        self.vth_ = (-front_left + front_right - rear_left + rear_right) / (2 * (self.wheel_separation_ + self.wheel_separation_length_))

    def publish_odometry(self):
        self.current_time_ = self.get_clock().now().to_msg()
        #delayed_time = self.get_clock().now() - rclpy.time.Duration(seconds=1.0)
        

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time_
        #odom_msg.header.stamp = delayed_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        ct = self.current_time_.sec + (self.current_time_.nanosec / 1e+9)
        lt = self.last_time_.sec + (self.last_time_.nanosec / 1e+9)
        dt = (ct - lt)

        # Update position
        delta_x = (self.vx_ * math.cos(self.theta_) - self.vy_ * math.sin(self.theta_)) * dt
        delta_y = (self.vx_ * math.sin(self.theta_) + self.vy_ * math.cos(self.theta_)) * dt
        delta_th = self.vth_ * dt

        self.x_ += delta_x
        self.y_ += delta_y
        self.theta_ += delta_th

        pose = Pose()
        pose.position.x = self.x_
        pose.position.y = self.y_
        pose.position.z = 0.0
        pose.orientation.z = math.sin(self.theta_ / 2)
        pose.orientation.w = math.cos(self.theta_ / 2)

        odom_msg.pose.pose = pose

        twist = Twist()
        twist.linear.x = self.vx_
        twist.linear.y = self.vy_
        twist.linear.z = 0.0
        twist.angular.z = self.vth_

        odom_msg.twist.twist = twist

        self.publisher_.publish(odom_msg)
        self.last_time_ = self.current_time_


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdometryPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
