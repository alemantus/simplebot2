// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ds4_driver:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__STATUS__TRAITS_HPP_
#define DS4_DRIVER__MSG__DETAIL__STATUS__TRAITS_HPP_

#include "ds4_driver/msg/detail/status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'imu'
#include "sensor_msgs/msg/detail/imu__traits.hpp"
// Member 'touch0'
// Member 'touch1'
#include "ds4_driver/msg/detail/trackpad__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ds4_driver::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: axis_left_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_left_x: ";
    value_to_yaml(msg.axis_left_x, out);
    out << "\n";
  }

  // member: axis_left_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_left_y: ";
    value_to_yaml(msg.axis_left_y, out);
    out << "\n";
  }

  // member: axis_right_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_right_x: ";
    value_to_yaml(msg.axis_right_x, out);
    out << "\n";
  }

  // member: axis_right_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_right_y: ";
    value_to_yaml(msg.axis_right_y, out);
    out << "\n";
  }

  // member: axis_l2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_l2: ";
    value_to_yaml(msg.axis_l2, out);
    out << "\n";
  }

  // member: axis_r2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_r2: ";
    value_to_yaml(msg.axis_r2, out);
    out << "\n";
  }

  // member: button_dpad_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_dpad_up: ";
    value_to_yaml(msg.button_dpad_up, out);
    out << "\n";
  }

  // member: button_dpad_down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_dpad_down: ";
    value_to_yaml(msg.button_dpad_down, out);
    out << "\n";
  }

  // member: button_dpad_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_dpad_left: ";
    value_to_yaml(msg.button_dpad_left, out);
    out << "\n";
  }

  // member: button_dpad_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_dpad_right: ";
    value_to_yaml(msg.button_dpad_right, out);
    out << "\n";
  }

  // member: button_cross
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_cross: ";
    value_to_yaml(msg.button_cross, out);
    out << "\n";
  }

  // member: button_circle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_circle: ";
    value_to_yaml(msg.button_circle, out);
    out << "\n";
  }

  // member: button_square
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_square: ";
    value_to_yaml(msg.button_square, out);
    out << "\n";
  }

  // member: button_triangle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_triangle: ";
    value_to_yaml(msg.button_triangle, out);
    out << "\n";
  }

  // member: button_l1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_l1: ";
    value_to_yaml(msg.button_l1, out);
    out << "\n";
  }

  // member: button_l2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_l2: ";
    value_to_yaml(msg.button_l2, out);
    out << "\n";
  }

  // member: button_l3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_l3: ";
    value_to_yaml(msg.button_l3, out);
    out << "\n";
  }

  // member: button_r1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_r1: ";
    value_to_yaml(msg.button_r1, out);
    out << "\n";
  }

  // member: button_r2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_r2: ";
    value_to_yaml(msg.button_r2, out);
    out << "\n";
  }

  // member: button_r3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_r3: ";
    value_to_yaml(msg.button_r3, out);
    out << "\n";
  }

  // member: button_share
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_share: ";
    value_to_yaml(msg.button_share, out);
    out << "\n";
  }

  // member: button_options
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_options: ";
    value_to_yaml(msg.button_options, out);
    out << "\n";
  }

  // member: button_trackpad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_trackpad: ";
    value_to_yaml(msg.button_trackpad, out);
    out << "\n";
  }

  // member: button_ps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_ps: ";
    value_to_yaml(msg.button_ps, out);
    out << "\n";
  }

  // member: imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu:\n";
    to_yaml(msg.imu, out, indentation + 2);
  }

  // member: battery_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_percentage: ";
    value_to_yaml(msg.battery_percentage, out);
    out << "\n";
  }

  // member: battery_full_charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_full_charging: ";
    value_to_yaml(msg.battery_full_charging, out);
    out << "\n";
  }

  // member: touch0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "touch0:\n";
    to_yaml(msg.touch0, out, indentation + 2);
  }

  // member: touch1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "touch1:\n";
    to_yaml(msg.touch1, out, indentation + 2);
  }

  // member: plug_usb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plug_usb: ";
    value_to_yaml(msg.plug_usb, out);
    out << "\n";
  }

  // member: plug_audio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plug_audio: ";
    value_to_yaml(msg.plug_audio, out);
    out << "\n";
  }

  // member: plug_mic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plug_mic: ";
    value_to_yaml(msg.plug_mic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ds4_driver::msg::Status & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ds4_driver::msg::Status>()
{
  return "ds4_driver::msg::Status";
}

template<>
inline const char * name<ds4_driver::msg::Status>()
{
  return "ds4_driver/msg/Status";
}

template<>
struct has_fixed_size<ds4_driver::msg::Status>
  : std::integral_constant<bool, has_fixed_size<ds4_driver::msg::Trackpad>::value && has_fixed_size<sensor_msgs::msg::Imu>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ds4_driver::msg::Status>
  : std::integral_constant<bool, has_bounded_size<ds4_driver::msg::Trackpad>::value && has_bounded_size<sensor_msgs::msg::Imu>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ds4_driver::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DS4_DRIVER__MSG__DETAIL__STATUS__TRAITS_HPP_
