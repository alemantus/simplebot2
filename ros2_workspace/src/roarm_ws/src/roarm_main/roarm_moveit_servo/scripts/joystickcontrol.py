#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Float32
from roarm_msgs.srv import ServoCommandType


class JoystickServo(Node):
    def __init__(self):
        super().__init__("joystick_servo")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("model", "roarm_m2")
        self.declare_parameter("enable_twist", True)
        self.declare_parameter("enable_joint", True)
        self.declare_parameter("enable_gripper", True)
        self.declare_parameter("twist_scale_lin", 0.3)
        self.declare_parameter("twist_scale_ang", 0.6)
        self.declare_parameter("joint_vel_scale", 1.0)
        self.declare_parameter("gripper_step", 0.02)

        model = self.get_parameter("model").value
        self.enable_twist = self.get_parameter("enable_twist").value
        self.enable_joint = self.get_parameter("enable_joint").value
        self.enable_gripper = self.get_parameter("enable_gripper").value
        self.twist_scale_lin = self.get_parameter("twist_scale_lin").value
        self.twist_scale_ang = self.get_parameter("twist_scale_ang").value
        self.joint_vel_scale = self.get_parameter("joint_vel_scale").value
        self.gripper_step = self.get_parameter("gripper_step").value

        # Set joint names based on model
        if model == "roarm_m2":
            self.joint_names = [
                "base_link_to_link1",
                "link1_to_link2",
                "link2_to_link3"
            ]
        elif model == "roarm_m3":
            self.joint_names = [
                "base_link_to_link1",
                "link1_to_link2",
                "link2_to_link3",
                "link3_to_link4",
                "link4_to_link5"
            ]
        else:
            self.get_logger().warn(f"Unknown model '{model}', defaulting to M3")
            self.joint_names = [
                "base_link_to_link1",
                "link1_to_link2",
                "link2_to_link3",
                "link3_to_link4",
                "link4_to_link5"
            ]

        self.get_logger().info(f"Using joint names: {self.joint_names}")

        # ---------------------------
        # Publishers
        # ---------------------------
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.gripper_pub = self.create_publisher(Float32, "/gripper_cmd", 10)

        # ---------------------------
        # Service client for mode switching
        # ---------------------------
        self.switch_srv = self.create_client(ServoCommandType, "/servo_node/switch_command_type")

        # ---------------------------
        # Joystick subscription
        # ---------------------------
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # ---------------------------
        # Internal state
        # ---------------------------
        self.current_frame = "base_link"
        self.gripper_pos = 0.5

        self.get_logger().info("PS4 joystick servo control started.")

    # ---------------------------
    # Mode switching
    # ---------------------------
    def switch_mode(self, mode_value: int):
        if not self.switch_srv.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("Servo switch service not available")
            return
        req = ServoCommandType.Request()
        req.command_type = mode_value
        self.switch_srv.call_async(req)

    # ---------------------------
    # Joystick callback
    # ---------------------------
    def joy_callback(self, msg: Joy):
        # ---- Buttons ----
        cross = msg.buttons[0]       # X — joint mode
        circle = msg.buttons[1]      # O — twist mode
        square = msg.buttons[2]      # □ — close gripper
        triangle = msg.buttons[3]    # △ — open gripper
        L1 = msg.buttons[4]
        R1 = msg.buttons[5]
        share = msg.buttons[8]       # frame = hand_tcp
        options = msg.buttons[9]     # frame = base_link

        # ---- Axes ----
        LX = msg.axes[0]
        LY = msg.axes[1]
        RX = msg.axes[2]
        RY = msg.axes[3]

        # ---------------------------
        # Switch modes
        # ---------------------------
        if circle:
            self.switch_mode(ServoCommandType.Request.TWIST)
        if cross:
            self.switch_mode(ServoCommandType.Request.JOINT_JOG)

        # ---------------------------
        # Switch command frame
        # ---------------------------
        if share:
            self.current_frame = "hand_tcp"
        if options:
            self.current_frame = "base_link"

        # ---------------------------
        # Twist commands
        # ---------------------------
        if self.enable_twist:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = self.current_frame

            twist.twist.linear.x = LY * self.twist_scale_lin
            twist.twist.linear.y = LX * self.twist_scale_lin
            twist.twist.angular.z = RX * self.twist_scale_ang
            twist.twist.linear.z = 0.0
            if L1:
                twist.twist.linear.z = 0.5
            elif R1:
                twist.twist.linear.z = -0.5

            self.twist_pub.publish(twist)

        # ---------------------------
        # Joint jog commands
        # ---------------------------
        if self.enable_joint:
            joint_msg = JointJog()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = self.current_frame
            joint_msg.joint_names = self.joint_names
            joint_msg.velocities = [0.0] * len(self.joint_names)

            if len(self.joint_names) > 0:
                joint_msg.velocities[0] = RX * self.joint_vel_scale
            if len(self.joint_names) > 1:
                joint_msg.velocities[1] = RY * self.joint_vel_scale

            self.joint_pub.publish(joint_msg)

        # ---------------------------
        # Gripper commands
        # ---------------------------
        if self.enable_gripper:
            if triangle:  # open
                self.gripper_pos += self.gripper_step
            if square:    # close
                self.gripper_pos -= self.gripper_step

            self.gripper_pos = max(0.0, min(1.5, self.gripper_pos))
            gr_msg = Float32()
            gr_msg.data = self.gripper_pos
            self.gripper_pub.publish(gr_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickServo()
    rclpy.spin(node)
    node.destr


if __name__ == "__main__":
    main()
