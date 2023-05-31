// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from img_transform:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__SRV__DETAIL__TELEPORT__BUILDER_HPP_
#define IMG_TRANSFORM__SRV__DETAIL__TELEPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "img_transform/srv/detail/teleport__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace img_transform
{

namespace srv
{

namespace builder
{

class Init_Teleport_Request_theta
{
public:
  explicit Init_Teleport_Request_theta(::img_transform::srv::Teleport_Request & msg)
  : msg_(msg)
  {}
  ::img_transform::srv::Teleport_Request theta(::img_transform::srv::Teleport_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::img_transform::srv::Teleport_Request msg_;
};

class Init_Teleport_Request_y
{
public:
  explicit Init_Teleport_Request_y(::img_transform::srv::Teleport_Request & msg)
  : msg_(msg)
  {}
  Init_Teleport_Request_theta y(::img_transform::srv::Teleport_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Teleport_Request_theta(msg_);
  }

private:
  ::img_transform::srv::Teleport_Request msg_;
};

class Init_Teleport_Request_x
{
public:
  Init_Teleport_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Teleport_Request_y x(::img_transform::srv::Teleport_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Teleport_Request_y(msg_);
  }

private:
  ::img_transform::srv::Teleport_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::img_transform::srv::Teleport_Request>()
{
  return img_transform::srv::builder::Init_Teleport_Request_x();
}

}  // namespace img_transform


namespace img_transform
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::img_transform::srv::Teleport_Response>()
{
  return ::img_transform::srv::Teleport_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace img_transform

#endif  // IMG_TRANSFORM__SRV__DETAIL__TELEPORT__BUILDER_HPP_
