// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ds4_driver:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
#define DS4_DRIVER__MSG__DETAIL__FEEDBACK__TRAITS_HPP_

#include "ds4_driver/msg/detail/feedback__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ds4_driver::msg::Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: set_led
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_led: ";
    value_to_yaml(msg.set_led, out);
    out << "\n";
  }

  // member: led_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_r: ";
    value_to_yaml(msg.led_r, out);
    out << "\n";
  }

  // member: led_g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_g: ";
    value_to_yaml(msg.led_g, out);
    out << "\n";
  }

  // member: led_b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_b: ";
    value_to_yaml(msg.led_b, out);
    out << "\n";
  }

  // member: set_led_flash
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_led_flash: ";
    value_to_yaml(msg.set_led_flash, out);
    out << "\n";
  }

  // member: led_flash_on
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_flash_on: ";
    value_to_yaml(msg.led_flash_on, out);
    out << "\n";
  }

  // member: led_flash_off
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_flash_off: ";
    value_to_yaml(msg.led_flash_off, out);
    out << "\n";
  }

  // member: set_rumble
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_rumble: ";
    value_to_yaml(msg.set_rumble, out);
    out << "\n";
  }

  // member: rumble_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rumble_duration: ";
    value_to_yaml(msg.rumble_duration, out);
    out << "\n";
  }

  // member: rumble_small
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rumble_small: ";
    value_to_yaml(msg.rumble_small, out);
    out << "\n";
  }

  // member: rumble_big
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rumble_big: ";
    value_to_yaml(msg.rumble_big, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ds4_driver::msg::Feedback & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ds4_driver::msg::Feedback>()
{
  return "ds4_driver::msg::Feedback";
}

template<>
inline const char * name<ds4_driver::msg::Feedback>()
{
  return "ds4_driver/msg/Feedback";
}

template<>
struct has_fixed_size<ds4_driver::msg::Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ds4_driver::msg::Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ds4_driver::msg::Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DS4_DRIVER__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
