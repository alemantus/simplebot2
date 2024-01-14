// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ds4_driver:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__STATUS__STRUCT_H_
#define DS4_DRIVER__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'imu'
#include "sensor_msgs/msg/detail/imu__struct.h"
// Member 'touch0'
// Member 'touch1'
#include "ds4_driver/msg/detail/trackpad__struct.h"

// Struct defined in msg/Status in the package ds4_driver.
typedef struct ds4_driver__msg__Status
{
  std_msgs__msg__Header header;
  float axis_left_x;
  float axis_left_y;
  float axis_right_x;
  float axis_right_y;
  float axis_l2;
  float axis_r2;
  int32_t button_dpad_up;
  int32_t button_dpad_down;
  int32_t button_dpad_left;
  int32_t button_dpad_right;
  int32_t button_cross;
  int32_t button_circle;
  int32_t button_square;
  int32_t button_triangle;
  int32_t button_l1;
  int32_t button_l2;
  int32_t button_l3;
  int32_t button_r1;
  int32_t button_r2;
  int32_t button_r3;
  int32_t button_share;
  int32_t button_options;
  int32_t button_trackpad;
  int32_t button_ps;
  sensor_msgs__msg__Imu imu;
  float battery_percentage;
  int32_t battery_full_charging;
  ds4_driver__msg__Trackpad touch0;
  ds4_driver__msg__Trackpad touch1;
  int32_t plug_usb;
  int32_t plug_audio;
  int32_t plug_mic;
} ds4_driver__msg__Status;

// Struct for a sequence of ds4_driver__msg__Status.
typedef struct ds4_driver__msg__Status__Sequence
{
  ds4_driver__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ds4_driver__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DS4_DRIVER__MSG__DETAIL__STATUS__STRUCT_H_
