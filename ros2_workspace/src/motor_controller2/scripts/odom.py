#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation as R

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
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
        self.wheel_separation_ = 0.222  # Distance between wheels (meters)
        self.wheel_separation_length_ = 0.13  # Distance between front and rear wheels (meters)

        # self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

    def encoder_callback(self, msg):
        # Assuming the encoder message has 4 elements: [front_left, front_right, rear_left, rear_right]
        front_left, front_right, rear_left, rear_right = msg.data
        self.front_left_debug = front_left
        self.front_right_debug = front_right
        # Update linear velocities directly from encoder readings
        #self.vx_ = (front_left + front_right + rear_left + rear_right) * 0.25  # Average of all wheels
        #self.vy_ = (front_left - front_right - rear_left + rear_right) * 0.25  # Average of opposite wheels

        # Calculate angular velocity
        #self.vth_ = (-front_left + front_right - rear_left + rear_right) / (2 * (self.wheel_separation_ + self.wheel_separation_length_))

        self.vx_ = (front_left + front_right + rear_left + rear_right) * 0.25  # Forward/backward
        self.vy_ = (-front_left + front_right + rear_left - rear_right) * 0.25  # Left/right
        self.vth_ = (-front_left + front_right - rear_left + rear_right) / (2 * (self.wheel_separation_ + self.wheel_separation_length_))

        self.publish_odometry()
        
    def publish_odometry(self):
        self.current_time_ = self.get_clock().now().to_msg()

        ct = self.current_time_.sec + (self.current_time_.nanosec/1e+9)
        lt = self.last_time_.sec + (self.last_time_.nanosec/1e+9)
        dt = (ct - lt)

        #dt = (self.current_time_ - self.last_time_).nanoseconds / 1e9

        # Update position
        delta_x = (self.vx_ * math.cos(self.theta_) - self.vy_ * math.sin(self.theta_)) * dt
        delta_y = (self.vx_ * math.sin(self.theta_) + self.vy_ * math.cos(self.theta_)) * dt
        delta_th = self.vth_ * dt

        self.x_ += delta_x
        self.y_ += delta_y
        self.theta_ += delta_th

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time_
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        pose = Pose()
        pose.position.x = self.x_
        pose.position.y = self.y_
        pose.position.z = 0.0

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        # may solve front left wheel spinning out of control. 24/08/2024
        try:
            pose.orientation.z = math.sin(self.theta_ / 2)
        except ValueError:
            pose.orientation.z = 0.0  # or some other default or fallback value
            print("Invalid theta_, setting pose.orientation.z to 0.")
        # may solve front left wheel spinning out of control. 24/08/2024
        try:
            pose.orientation.w = math.cos(self.theta_ / 2)
        except ValueError:
            pose.orientation.w = 0.0  # or some other default or fallback value
            print(f"theta: {self.theta_}")
            print(f"front right: {self.front_right_debug}")
            print(f"front left: {self.front_left_debug}")
            print("Invalid theta_, setting pose.orientation.w to 0.")

        odom_msg.pose.pose = pose

        
        twist = Twist()
        twist.linear.x = self.vx_
        twist.linear.y = self.vy_
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.vth_

        odom_msg.twist.twist = twist

        self.publisher_.publish(odom_msg)

        # Broadcast transform
        # transform = TransformStamped()
        # transform.header.stamp = self.current_time_
        # transform.header.frame_id = 'odom'
        # transform.child_frame_id = 'base_link'
        # transform.transform.translation.x = self.x_
        # transform.transform.translation.y = self.y_
        # transform.transform.translation.z = 0.0
        # r = R.from_euler('xyz',[0, 0, self.theta_])
 
        # transform.transform.rotation.x = r.as_quat()[0]
        # transform.transform.rotation.y = r.as_quat()[1]
        # transform.transform.rotation.z = r.as_quat()[2]
        # transform.transform.rotation.w = r.as_quat()[3]
 
         #self.tf_broadcaster_.sendTransform(transform)

        self.last_time_ = self.current_time_


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdometryPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
