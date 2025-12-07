#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class MecanumJointStatePublisher(Node):
    def __init__(self):
        super().__init__('mecanum_joint_state_publisher')
        
        self.conversion_factor = 2.0 * math.pi # Factor to convert RPS to rad/s (units fix)

        # Joint names order in the JointState message: [FL, FR, RL, RR]
        self.joint_names = [
            "Revolute 1",  # 0: Front Left
            "Revolute 4",  # 1: Front Right
            "Revolute 2",  # 2: Rear Left
            "Revolute 3"   # 3: Rear Right
        ]
        
        self.positions = [0.0, 0.0, 0.0, 0.0]
        self.last_time = None

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/encoder_data',
            self.encoder_callback,
            10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("✓ Mecanum JointState Publisher running")

    def encoder_callback(self, msg):
        # ... (error handling and previous time checks)

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0:
            return

        # FL Joint (Revolute 1) is driven by RR data (msg.data[3])
        fl_ang_vel = msg.data[3] * self.conversion_factor
        
        # FR Joint (Revolute 4) is driven by FL data (msg.data[2])
        fr_ang_vel = msg.data[2] * self.conversion_factor
        
        # RL Joint (Revolute 2) is driven by FR data (msg.data[1])
        rl_ang_vel = msg.data[1] * self.conversion_factor
        
        # RR Joint (Revolute 3) is driven by RL data (msg.data[0])
        rr_ang_vel = msg.data[0] * self.conversion_factor
        
        # 2. FINAL SIGN APPLICATION: Applying the required [+FL, +FR, +RL, +RR] pattern 
        # (This pattern ensures all wheels rotate forward when you command forward motion).
        
        velocities = [
            # 0: FL (Revolute 1): PLUS (Required sign)
            fl_ang_vel,
            # 1: FR (Revolute 4): PLUS (Required sign)
            fr_ang_vel,
            # 2: RL (Revolute 2): PLUS (Required sign)
            rl_ang_vel,
            # 3: RR (Revolute 3): PLUS (Required sign)
            rr_ang_vel
        ]

        # 3. INTEGRATION (Fixes Jumping: No wrapping/modulo logic)
        for i in range(4):
            self.positions[i] += velocities[i] * dt

        # Publish JointState
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = self.positions
        js.velocity = velocities

        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()