// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ds4_driver:msg/Trackpad.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__TRACKPAD__TRAITS_HPP_
#define DS4_DRIVER__MSG__DETAIL__TRACKPAD__TRAITS_HPP_

#include "ds4_driver/msg/detail/trackpad__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const ds4_driver::msg::Trackpad & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    value_to_yaml(msg.active, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ds4_driver::msg::Trackpad & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<ds4_driver::msg::Trackpad>()
{
  return "ds4_driver::msg::Trackpad";
}

template<>
inline const char * name<ds4_driver::msg::Trackpad>()
{
  return "ds4_driver/msg/Trackpad";
}

template<>
struct has_fixed_size<ds4_driver::msg::Trackpad>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ds4_driver::msg::Trackpad>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ds4_driver::msg::Trackpad>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DS4_DRIVER__MSG__DETAIL__TRACKPAD__TRAITS_HPP_
