// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ds4_driver:msg/Report.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__REPORT__STRUCT_H_
#define DS4_DRIVER__MSG__DETAIL__REPORT__STRUCT_H_

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

// Struct defined in msg/Report in the package ds4_driver.
typedef struct ds4_driver__msg__Report
{
  std_msgs__msg__Header header;
  uint8_t left_analog_x;
  uint8_t left_analog_y;
  uint8_t right_analog_x;
  uint8_t right_analog_y;
  uint8_t l2_analog;
  uint8_t r2_analog;
  bool dpad_up;
  bool dpad_down;
  bool dpad_left;
  bool dpad_right;
  bool button_cross;
  bool button_circle;
  bool button_square;
  bool button_triangle;
  bool button_l1;
  bool button_l2;
  bool button_l3;
  bool button_r1;
  bool button_r2;
  bool button_r3;
  bool button_share;
  bool button_options;
  bool button_trackpad;
  bool button_ps;
  int16_t lin_acc_x;
  int16_t lin_acc_y;
  int16_t lin_acc_z;
  int16_t ang_vel_x;
  int16_t ang_vel_y;
  int16_t ang_vel_z;
  uint16_t trackpad_touch0_id;
  uint16_t trackpad_touch0_active;
  uint16_t trackpad_touch0_x;
  uint16_t trackpad_touch0_y;
  uint16_t trackpad_touch1_id;
  uint16_t trackpad_touch1_active;
  uint16_t trackpad_touch1_x;
  uint16_t trackpad_touch1_y;
  uint8_t timestamp;
  uint8_t battery;
  bool plug_usb;
  bool plug_audio;
  bool plug_mic;
} ds4_driver__msg__Report;

// Struct for a sequence of ds4_driver__msg__Report.
typedef struct ds4_driver__msg__Report__Sequence
{
  ds4_driver__msg__Report * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ds4_driver__msg__Report__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DS4_DRIVER__MSG__DETAIL__REPORT__STRUCT_H_
