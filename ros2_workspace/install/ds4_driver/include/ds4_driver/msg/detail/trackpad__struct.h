// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ds4_driver:msg/Trackpad.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__TRACKPAD__STRUCT_H_
#define DS4_DRIVER__MSG__DETAIL__TRACKPAD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Trackpad in the package ds4_driver.
typedef struct ds4_driver__msg__Trackpad
{
  uint16_t id;
  int32_t active;
  float x;
  float y;
} ds4_driver__msg__Trackpad;

// Struct for a sequence of ds4_driver__msg__Trackpad.
typedef struct ds4_driver__msg__Trackpad__Sequence
{
  ds4_driver__msg__Trackpad * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ds4_driver__msg__Trackpad__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DS4_DRIVER__MSG__DETAIL__TRACKPAD__STRUCT_H_
