// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_define:srv/SetString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "service_define/srv/set_string.hpp"


#ifndef SERVICE_DEFINE__SRV__DETAIL__SET_STRING__BUILDER_HPP_
#define SERVICE_DEFINE__SRV__DETAIL__SET_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_define/srv/detail/set_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_define
{

namespace srv
{

namespace builder
{

class Init_SetString_Request_data
{
public:
  Init_SetString_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_define::srv::SetString_Request data(::service_define::srv::SetString_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_define::srv::SetString_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_define::srv::SetString_Request>()
{
  return service_define::srv::builder::Init_SetString_Request_data();
}

}  // namespace service_define


namespace service_define
{

namespace srv
{

namespace builder
{

class Init_SetString_Response_success
{
public:
  Init_SetString_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_define::srv::SetString_Response success(::service_define::srv::SetString_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_define::srv::SetString_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_define::srv::SetString_Response>()
{
  return service_define::srv::builder::Init_SetString_Response_success();
}

}  // namespace service_define


namespace service_define
{

namespace srv
{

namespace builder
{

class Init_SetString_Event_response
{
public:
  explicit Init_SetString_Event_response(::service_define::srv::SetString_Event & msg)
  : msg_(msg)
  {}
  ::service_define::srv::SetString_Event response(::service_define::srv::SetString_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_define::srv::SetString_Event msg_;
};

class Init_SetString_Event_request
{
public:
  explicit Init_SetString_Event_request(::service_define::srv::SetString_Event & msg)
  : msg_(msg)
  {}
  Init_SetString_Event_response request(::service_define::srv::SetString_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetString_Event_response(msg_);
  }

private:
  ::service_define::srv::SetString_Event msg_;
};

class Init_SetString_Event_info
{
public:
  Init_SetString_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetString_Event_request info(::service_define::srv::SetString_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetString_Event_request(msg_);
  }

private:
  ::service_define::srv::SetString_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_define::srv::SetString_Event>()
{
  return service_define::srv::builder::Init_SetString_Event_info();
}

}  // namespace service_define

#endif  // SERVICE_DEFINE__SRV__DETAIL__SET_STRING__BUILDER_HPP_
