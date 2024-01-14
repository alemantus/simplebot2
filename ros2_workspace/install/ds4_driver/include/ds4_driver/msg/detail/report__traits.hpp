// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ds4_driver:msg/Report.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__REPORT__TRAITS_HPP_
#define DS4_DRIVER__MSG__DETAIL__REPORT__TRAITS_HPP_

#include "ds4_driver/msg/detail/report__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ds4_driver::msg::Report & msg,
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

  // member: left_analog_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_analog_x: ";
    value_to_yaml(msg.left_analog_x, out);
    out << "\n";
  }

  // member: left_analog_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_analog_y: ";
    value_to_yaml(msg.left_analog_y, out);
    out << "\n";
  }

  // member: right_analog_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_analog_x: ";
    value_to_yaml(msg.right_analog_x, out);
    out << "\n";
  }

  // member: right_analog_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_analog_y: ";
    value_to_yaml(msg.right_analog_y, out);
    out << "\n";
  }

  // member: l2_analog
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l2_analog: ";
    value_to_yaml(msg.l2_analog, out);
    out << "\n";
  }

  // member: r2_analog
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r2_analog: ";
    value_to_yaml(msg.r2_analog, out);
    out << "\n";
  }

  // member: dpad_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dpad_up: ";
    value_to_yaml(msg.dpad_up, out);
    out << "\n";
  }

  // member: dpad_down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dpad_down: ";
    value_to_yaml(msg.dpad_down, out);
    out << "\n";
  }

  // member: dpad_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dpad_left: ";
    value_to_yaml(msg.dpad_left, out);
    out << "\n";
  }

  // member: dpad_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dpad_right: ";
    value_to_yaml(msg.dpad_right, out);
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

  // member: lin_acc_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lin_acc_x: ";
    value_to_yaml(msg.lin_acc_x, out);
    out << "\n";
  }

  // member: lin_acc_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lin_acc_y: ";
    value_to_yaml(msg.lin_acc_y, out);
    out << "\n";
  }

  // member: lin_acc_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lin_acc_z: ";
    value_to_yaml(msg.lin_acc_z, out);
    out << "\n";
  }

  // member: ang_vel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_vel_x: ";
    value_to_yaml(msg.ang_vel_x, out);
    out << "\n";
  }

  // member: ang_vel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_vel_y: ";
    value_to_yaml(msg.ang_vel_y, out);
    out << "\n";
  }

  // member: ang_vel_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_vel_z: ";
    value_to_yaml(msg.ang_vel_z, out);
    out << "\n";
  }

  // member: trackpad_touch0_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch0_id: ";
    value_to_yaml(msg.trackpad_touch0_id, out);
    out << "\n";
  }

  // member: trackpad_touch0_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch0_active: ";
    value_to_yaml(msg.trackpad_touch0_active, out);
    out << "\n";
  }

  // member: trackpad_touch0_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch0_x: ";
    value_to_yaml(msg.trackpad_touch0_x, out);
    out << "\n";
  }

  // member: trackpad_touch0_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch0_y: ";
    value_to_yaml(msg.trackpad_touch0_y, out);
    out << "\n";
  }

  // member: trackpad_touch1_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch1_id: ";
    value_to_yaml(msg.trackpad_touch1_id, out);
    out << "\n";
  }

  // member: trackpad_touch1_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch1_active: ";
    value_to_yaml(msg.trackpad_touch1_active, out);
    out << "\n";
  }

  // member: trackpad_touch1_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch1_x: ";
    value_to_yaml(msg.trackpad_touch1_x, out);
    out << "\n";
  }

  // member: trackpad_touch1_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trackpad_touch1_y: ";
    value_to_yaml(msg.trackpad_touch1_y, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery: ";
    value_to_yaml(msg.battery, out);
    out << "\n";
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

inline std::string to_yaml(const ds4_driver::msg::Report & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ds4_driver::msg::Report>()
{
  return "ds4_driver::msg::Report";
}

template<>
inline const char * name<ds4_driver::msg::Report>()
{
  return "ds4_driver/msg/Report";
}

template<>
struct has_fixed_size<ds4_driver::msg::Report>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ds4_driver::msg::Report>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ds4_driver::msg::Report>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DS4_DRIVER__MSG__DETAIL__REPORT__TRAITS_HPP_
