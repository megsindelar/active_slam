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

class Init_Keypoints_class_id_2
{
public:
  explicit Init_Keypoints_class_id_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  ::turtlebot_control::msg::Keypoints class_id_2(::turtlebot_control::msg::Keypoints::_class_id_2_type arg)
  {
    msg_.class_id_2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_octave_2
{
public:
  explicit Init_Keypoints_octave_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_class_id_2 octave_2(::turtlebot_control::msg::Keypoints::_octave_2_type arg)
  {
    msg_.octave_2 = std::move(arg);
    return Init_Keypoints_class_id_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_response_2
{
public:
  explicit Init_Keypoints_response_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_octave_2 response_2(::turtlebot_control::msg::Keypoints::_response_2_type arg)
  {
    msg_.response_2 = std::move(arg);
    return Init_Keypoints_octave_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_angle_2
{
public:
  explicit Init_Keypoints_angle_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_response_2 angle_2(::turtlebot_control::msg::Keypoints::_angle_2_type arg)
  {
    msg_.angle_2 = std::move(arg);
    return Init_Keypoints_response_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_size_2
{
public:
  explicit Init_Keypoints_size_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_angle_2 size_2(::turtlebot_control::msg::Keypoints::_size_2_type arg)
  {
    msg_.size_2 = std::move(arg);
    return Init_Keypoints_angle_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_y_2
{
public:
  explicit Init_Keypoints_y_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_size_2 y_2(::turtlebot_control::msg::Keypoints::_y_2_type arg)
  {
    msg_.y_2 = std::move(arg);
    return Init_Keypoints_size_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_x_2
{
public:
  explicit Init_Keypoints_x_2(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_y_2 x_2(::turtlebot_control::msg::Keypoints::_x_2_type arg)
  {
    msg_.x_2 = std::move(arg);
    return Init_Keypoints_y_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_class_id_1
{
public:
  explicit Init_Keypoints_class_id_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_x_2 class_id_1(::turtlebot_control::msg::Keypoints::_class_id_1_type arg)
  {
    msg_.class_id_1 = std::move(arg);
    return Init_Keypoints_x_2(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_octave_1
{
public:
  explicit Init_Keypoints_octave_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_class_id_1 octave_1(::turtlebot_control::msg::Keypoints::_octave_1_type arg)
  {
    msg_.octave_1 = std::move(arg);
    return Init_Keypoints_class_id_1(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_response_1
{
public:
  explicit Init_Keypoints_response_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_octave_1 response_1(::turtlebot_control::msg::Keypoints::_response_1_type arg)
  {
    msg_.response_1 = std::move(arg);
    return Init_Keypoints_octave_1(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_angle_1
{
public:
  explicit Init_Keypoints_angle_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_response_1 angle_1(::turtlebot_control::msg::Keypoints::_angle_1_type arg)
  {
    msg_.angle_1 = std::move(arg);
    return Init_Keypoints_response_1(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_size_1
{
public:
  explicit Init_Keypoints_size_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_angle_1 size_1(::turtlebot_control::msg::Keypoints::_size_1_type arg)
  {
    msg_.size_1 = std::move(arg);
    return Init_Keypoints_angle_1(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_y_1
{
public:
  explicit Init_Keypoints_y_1(::turtlebot_control::msg::Keypoints & msg)
  : msg_(msg)
  {}
  Init_Keypoints_size_1 y_1(::turtlebot_control::msg::Keypoints::_y_1_type arg)
  {
    msg_.y_1 = std::move(arg);
    return Init_Keypoints_size_1(msg_);
  }

private:
  ::turtlebot_control::msg::Keypoints msg_;
};

class Init_Keypoints_x_1
{
public:
  Init_Keypoints_x_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Keypoints_y_1 x_1(::turtlebot_control::msg::Keypoints::_x_1_type arg)
  {
    msg_.x_1 = std::move(arg);
    return Init_Keypoints_y_1(msg_);
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
  return turtlebot_control::msg::builder::Init_Keypoints_x_1();
}

}  // namespace turtlebot_control

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__BUILDER_HPP_
