#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float32.hpp>
#include <roarm_msgs/srv/servo_command_type.hpp>

class JoystickServo : public rclcpp::Node
{
public:
  JoystickServo() : Node("joystick_control"), joint_vel_cmd_(1.0), gripper_value_(0.5)
  {
    // Publishers
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
        "/servo_node/delta_joint_cmds", 10);

    gripper_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/gripper_cmd", 10);

    // Mode switch service
    switch_input_ = create_client<roarm_msgs::srv::ServoCommandType>(
        "/servo_node/switch_command_type");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&JoystickServo::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PS4 Joystick Servo Control Started");
  }

private:
  // ---- Callback handling joystick input ----
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // ------------- Buttons ---------------
    bool btn_cross = msg->buttons[0];
    bool btn_circle = msg->buttons[1];
    bool btn_square = msg->buttons[2];
    bool btn_triangle = msg->buttons[3];

    bool btn_L1 = msg->buttons[4];
    bool btn_R1 = msg->buttons[5];

    bool btn_share = msg->buttons[8];
    bool btn_options = msg->buttons[9];

    // ------------- Axes (sticks) ----------
    double LX = msg->axes[0];
    double LY = msg->axes[1];
    double RX = msg->axes[2];
    double RY = msg->axes[3];
    double L2 = msg->axes[4];   // usually analog
    double R2 = msg->axes[5];

    // Example mapping:
    // Left stick  -> linear motion
    // Right stick -> rotation
    // L1+R1 -> Z axis up/down
    // ◯ → TWIST mode
    // ❌ → JOINT mode
    // SHARE → EE frame, OPTIONS → Base frame

    // ----------------- Switch Mode -----------------
    if (btn_circle)
      switch_mode(roarm_msgs::srv::ServoCommandType::Request::TWIST);

    if (btn_cross)
      switch_mode(roarm_msgs::srv::ServoCommandType::Request::JOINT_JOG);

    // ----------------- Frame Switch -----------------
    if (btn_share)
      current_frame_ = "hand_tcp";

    if (btn_options)
      current_frame_ = "base_link";

    // ----------------- Twist Command -----------------
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.stamp = now();
    twist.header.frame_id = current_frame_;

    twist.twist.linear.x = LY * 0.5;
    twist.twist.linear.y = LX * 0.5;
    twist.twist.angular.z = RX * 1.0;

    // L1 / R1 for Z movement
    if (btn_L1)
      twist.twist.linear.z = 0.5;
    else if (btn_R1)
      twist.twist.linear.z = -0.5;

    twist_pub_->publish(twist);

    // ----------------- Gripper -----------------
    if (btn_triangle)     // open
      gripper_value_ += 0.01;

    if (btn_square)       // close
      gripper_value_ -= 0.01;

    gripper_value_ = std::clamp(gripper_value_, 0.0, 1.5);

    auto g = std_msgs::msg::Float32();
    g.data = gripper_value_;
    gripper_pub_->publish(g);
  }

  // --------- Service to switch input type ----------
  void switch_mode(int mode)
  {
    if (!switch_input_->wait_for_service(std::chrono::milliseconds(10)))
      return;

    auto req = std::make_shared<roarm_msgs::srv::ServoCommandType::Request>();
    req->command_type = mode;
    switch_input_->async_send_request(req);
  }

  // -------- Members --------
  double joint_vel_cmd_;
  double gripper_value_;
  std::string current_frame_ = "base_link";

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  rclcpp::Client<roarm_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServo>());
  rclcpp::shutdown();
  return 0;
}
