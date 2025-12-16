// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from service_define:srv/StringForString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "service_define/srv/string_for_string.hpp"


#ifndef SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__TRAITS_HPP_
#define SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "service_define/srv/detail/string_for_string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace service_define
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringForString_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringForString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringForString_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_define

namespace rosidl_generator_traits
{

[[deprecated("use service_define::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_define::srv::StringForString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_define::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_define::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_define::srv::StringForString_Request & msg)
{
  return service_define::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_define::srv::StringForString_Request>()
{
  return "service_define::srv::StringForString_Request";
}

template<>
inline const char * name<service_define::srv::StringForString_Request>()
{
  return "service_define/srv/StringForString_Request";
}

template<>
struct has_fixed_size<service_define::srv::StringForString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<service_define::srv::StringForString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<service_define::srv::StringForString_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace service_define
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringForString_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringForString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringForString_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_define

namespace rosidl_generator_traits
{

[[deprecated("use service_define::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_define::srv::StringForString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_define::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_define::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_define::srv::StringForString_Response & msg)
{
  return service_define::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_define::srv::StringForString_Response>()
{
  return "service_define::srv::StringForString_Response";
}

template<>
inline const char * name<service_define::srv::StringForString_Response>()
{
  return "service_define/srv/StringForString_Response";
}

template<>
struct has_fixed_size<service_define::srv::StringForString_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<service_define::srv::StringForString_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<service_define::srv::StringForString_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace service_define
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringForString_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringForString_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringForString_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_define

namespace rosidl_generator_traits
{

[[deprecated("use service_define::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_define::srv::StringForString_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_define::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_define::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_define::srv::StringForString_Event & msg)
{
  return service_define::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_define::srv::StringForString_Event>()
{
  return "service_define::srv::StringForString_Event";
}

template<>
inline const char * name<service_define::srv::StringForString_Event>()
{
  return "service_define/srv/StringForString_Event";
}

template<>
struct has_fixed_size<service_define::srv::StringForString_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<service_define::srv::StringForString_Event>
  : std::integral_constant<bool, has_bounded_size<service_define::srv::StringForString_Request>::value && has_bounded_size<service_define::srv::StringForString_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<service_define::srv::StringForString_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<service_define::srv::StringForString>()
{
  return "service_define::srv::StringForString";
}

template<>
inline const char * name<service_define::srv::StringForString>()
{
  return "service_define/srv/StringForString";
}

template<>
struct has_fixed_size<service_define::srv::StringForString>
  : std::integral_constant<
    bool,
    has_fixed_size<service_define::srv::StringForString_Request>::value &&
    has_fixed_size<service_define::srv::StringForString_Response>::value
  >
{
};

template<>
struct has_bounded_size<service_define::srv::StringForString>
  : std::integral_constant<
    bool,
    has_bounded_size<service_define::srv::StringForString_Request>::value &&
    has_bounded_size<service_define::srv::StringForString_Response>::value
  >
{
};

template<>
struct is_service<service_define::srv::StringForString>
  : std::true_type
{
};

template<>
struct is_service_request<service_define::srv::StringForString_Request>
  : std::true_type
{
};

template<>
struct is_service_response<service_define::srv::StringForString_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SERVICE_DEFINE__SRV__DETAIL__STRING_FOR_STRING__TRAITS_HPP_
