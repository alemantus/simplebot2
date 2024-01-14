// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ds4_driver:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef DS4_DRIVER__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define DS4_DRIVER__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Feedback in the package ds4_driver.
typedef struct ds4_driver__msg__Feedback
{
  bool set_led;
  float led_r;
  float led_g;
  float led_b;
  bool set_led_flash;
  float led_flash_on;
  float led_flash_off;
  bool set_rumble;
  float rumble_duration;
  float rumble_small;
  float rumble_big;
} ds4_driver__msg__Feedback;

// Struct for a sequence of ds4_driver__msg__Feedback.
typedef struct ds4_driver__msg__Feedback__Sequence
{
  ds4_driver__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ds4_driver__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DS4_DRIVER__MSG__DETAIL__FEEDBACK__STRUCT_H_
