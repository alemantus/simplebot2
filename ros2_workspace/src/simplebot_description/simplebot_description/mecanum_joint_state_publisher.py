#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class MecanumJointStatePublisher(Node):
    def __init__(self):
        super().__init__('mecanum_joint_state_publisher')
        
        # Explicitly use simulation time
        self.declare_parameter('use_sim_time', True)

        # The names in your URDF
        self.target_joint_names = [
            "joint_front_left",
            "joint_front_right",
            "joint_rear_left",
            "joint_rear_right"
        ]

        # Subscription to the bridged Gazebo topic
        # Note: We changed the type to JointState
        self.subscription = self.create_subscription(
            JointState,
            '/encoder_data_gz',
            self.joint_callback,
            10
        )

        # Publisher to the standard ROS topic
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.get_logger().info("✓ Mecanum JointState Bridge (JointState -> JointState) Started")

    def joint_callback(self, msg):
        """
        Receives JointState from Gazebo and ensures it is published 
        correctly for robot_state_publisher.
        """
        
        # Create the outgoing message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.target_joint_names

        # If the Gazebo plugin provides joints in a different order,
        # you can remap them here. Assuming Gazebo sends them in the 
        # order defined in the URDF plugin:
        if len(msg.position) >= 4:
            # We pass the positions through. 
            # If your wheels spin the wrong way, multiply by -1.0 here.
            js.position = [
                msg.position[0], # Front Left
                msg.position[1], # Front Right
                msg.position[2], # Rear Left
                msg.position[3]  # Rear Right
            ]
            
            # Optional: Pass velocities through as well
            if len(msg.velocity) >= 4:
                js.velocity = msg.velocity

            self.publisher.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()