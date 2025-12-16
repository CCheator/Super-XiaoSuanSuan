// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from service_define:srv/StringForString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "service_define/srv/string_for_string.h"


#ifndef SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__STRUCT_H_
#define SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StringForString in the package service_define.
typedef struct service_define__srv__StringForString_Request
{
  rosidl_runtime_c__String data;
} service_define__srv__StringForString_Request;

// Struct for a sequence of service_define__srv__StringForString_Request.
typedef struct service_define__srv__StringForString_Request__Sequence
{
  service_define__srv__StringForString_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_define__srv__StringForString_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'data'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StringForString in the package service_define.
typedef struct service_define__srv__StringForString_Response
{
  rosidl_runtime_c__String data;
} service_define__srv__StringForString_Response;

// Struct for a sequence of service_define__srv__StringForString_Response.
typedef struct service_define__srv__StringForString_Response__Sequence
{
  service_define__srv__StringForString_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_define__srv__StringForString_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  service_define__srv__StringForString_Event__request__MAX_SIZE = 1
};
// response
enum
{
  service_define__srv__StringForString_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/StringForString in the package service_define.
typedef struct service_define__srv__StringForString_Event
{
  service_msgs__msg__ServiceEventInfo info;
  service_define__srv__StringForString_Request__Sequence request;
  service_define__srv__StringForString_Response__Sequence response;
} service_define__srv__StringForString_Event;

// Struct for a sequence of service_define__srv__StringForString_Event.
typedef struct service_define__srv__StringForString_Event__Sequence
{
  service_define__srv__StringForString_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} service_define__srv__StringForString_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__STRUCT_H_
