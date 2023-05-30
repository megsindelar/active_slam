// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__BUILDER_HPP_
#define TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot_control/msg/detail/keypoints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot_control
{

namespace msg
{

namespace builder
{

class Init_Keypoints_keypoints_2
{
public:
  explicit Init_Keypoints_keypoints_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  ::turtlebot_control::msg::Keypoints keypoints_2(::turtlebot_control::msg::Keypoints::_keypoints_2_type arg)
  {
    msg_.keypoints_2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_keypoints_1
{
public:
  Init_Keypoints_keypoints_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Keypoints_keypoints_2 keypoints_1(::turtlebot_control::msg::Keypoints::_keypoints_1_type arg)
  {
    msg_.keypoints_1 = std::move(arg);
    return Init_Keypoints_keypoints_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot_control::msg::Keypoints>()
{
  return turtlebot_control::msg::builder::Init_Keypoints_keypoints_1();
}

}  // namespace turtlebot_control

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__BUILDER_HPP_
