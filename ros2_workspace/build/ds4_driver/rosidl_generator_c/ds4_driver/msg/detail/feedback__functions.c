// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ds4_driver:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "ds4_driver/msg/detail/feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
ds4_driver__msg__Feedback__init(ds4_driver__msg__Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // set_led
  // led_r
  // led_g
  // led_b
  // set_led_flash
  // led_flash_on
  // led_flash_off
  // set_rumble
  // rumble_duration
  // rumble_small
  // rumble_big
  return true;
}

void
ds4_driver__msg__Feedback__fini(ds4_driver__msg__Feedback * msg)
{
  if (!msg) {
    return;
  }
  // set_led
  // led_r
  // led_g
  // led_b
  // set_led_flash
  // led_flash_on
  // led_flash_off
  // set_rumble
  // rumble_duration
  // rumble_small
  // rumble_big
}

bool
ds4_driver__msg__Feedback__are_equal(const ds4_driver__msg__Feedback * lhs, const ds4_driver__msg__Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // set_led
  if (lhs->set_led != rhs->set_led) {
    return false;
  }
  // led_r
  if (lhs->led_r != rhs->led_r) {
    return false;
  }
  // led_g
  if (lhs->led_g != rhs->led_g) {
    return false;
  }
  // led_b
  if (lhs->led_b != rhs->led_b) {
    return false;
  }
  // set_led_flash
  if (lhs->set_led_flash != rhs->set_led_flash) {
    return false;
  }
  // led_flash_on
  if (lhs->led_flash_on != rhs->led_flash_on) {
    return false;
  }
  // led_flash_off
  if (lhs->led_flash_off != rhs->led_flash_off) {
    return false;
  }
  // set_rumble
  if (lhs->set_rumble != rhs->set_rumble) {
    return false;
  }
  // rumble_duration
  if (lhs->rumble_duration != rhs->rumble_duration) {
    return false;
  }
  // rumble_small
  if (lhs->rumble_small != rhs->rumble_small) {
    return false;
  }
  // rumble_big
  if (lhs->rumble_big != rhs->rumble_big) {
    return false;
  }
  return true;
}

bool
ds4_driver__msg__Feedback__copy(
  const ds4_driver__msg__Feedback * input,
  ds4_driver__msg__Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // set_led
  output->set_led = input->set_led;
  // led_r
  output->led_r = input->led_r;
  // led_g
  output->led_g = input->led_g;
  // led_b
  output->led_b = input->led_b;
  // set_led_flash
  output->set_led_flash = input->set_led_flash;
  // led_flash_on
  output->led_flash_on = input->led_flash_on;
  // led_flash_off
  output->led_flash_off = input->led_flash_off;
  // set_rumble
  output->set_rumble = input->set_rumble;
  // rumble_duration
  output->rumble_duration = input->rumble_duration;
  // rumble_small
  output->rumble_small = input->rumble_small;
  // rumble_big
  output->rumble_big = input->rumble_big;
  return true;
}

ds4_driver__msg__Feedback *
ds4_driver__msg__Feedback__create()
{
  ds4_driver__msg__Feedback * msg = (ds4_driver__msg__Feedback *)malloc(sizeof(ds4_driver__msg__Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ds4_driver__msg__Feedback));
  bool success = ds4_driver__msg__Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ds4_driver__msg__Feedback__destroy(ds4_driver__msg__Feedback * msg)
{
  if (msg) {
    ds4_driver__msg__Feedback__fini(msg);
  }
  free(msg);
}


bool
ds4_driver__msg__Feedback__Sequence__init(ds4_driver__msg__Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ds4_driver__msg__Feedback * data = NULL;
  if (size) {
    data = (ds4_driver__msg__Feedback *)calloc(size, sizeof(ds4_driver__msg__Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ds4_driver__msg__Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ds4_driver__msg__Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ds4_driver__msg__Feedback__Sequence__fini(ds4_driver__msg__Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ds4_driver__msg__Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ds4_driver__msg__Feedback__Sequence *
ds4_driver__msg__Feedback__Sequence__create(size_t size)
{
  ds4_driver__msg__Feedback__Sequence * array = (ds4_driver__msg__Feedback__Sequence *)malloc(sizeof(ds4_driver__msg__Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ds4_driver__msg__Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ds4_driver__msg__Feedback__Sequence__destroy(ds4_driver__msg__Feedback__Sequence * array)
{
  if (array) {
    ds4_driver__msg__Feedback__Sequence__fini(array);
  }
  free(array);
}

bool
ds4_driver__msg__Feedback__Sequence__are_equal(const ds4_driver__msg__Feedback__Sequence * lhs, const ds4_driver__msg__Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ds4_driver__msg__Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ds4_driver__msg__Feedback__Sequence__copy(
  const ds4_driver__msg__Feedback__Sequence * input,
  ds4_driver__msg__Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ds4_driver__msg__Feedback);
    ds4_driver__msg__Feedback * data =
      (ds4_driver__msg__Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ds4_driver__msg__Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ds4_driver__msg__Feedback__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ds4_driver__msg__Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
