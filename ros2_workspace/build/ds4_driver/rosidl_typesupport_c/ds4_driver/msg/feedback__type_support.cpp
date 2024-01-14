// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ds4_driver:msg/Feedback.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ds4_driver/msg/rosidl_typesupport_c__visibility_control.h"
#include "ds4_driver/msg/detail/feedback__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ds4_driver
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Feedback_type_support_ids_t;

static const _Feedback_type_support_ids_t _Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Feedback_type_support_symbol_names_t _Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ds4_driver, msg, Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ds4_driver, msg, Feedback)),
  }
};

typedef struct _Feedback_type_support_data_t
{
  void * data[2];
} _Feedback_type_support_data_t;

static _Feedback_type_support_data_t _Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Feedback_message_typesupport_map = {
  2,
  "ds4_driver",
  &_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace ds4_driver

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_ds4_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ds4_driver, msg, Feedback)() {
  return &::ds4_driver::msg::rosidl_typesupport_c::Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
