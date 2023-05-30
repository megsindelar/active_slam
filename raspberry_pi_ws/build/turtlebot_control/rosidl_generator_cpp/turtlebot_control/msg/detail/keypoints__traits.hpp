// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__TRAITS_HPP_
#define TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot_control/msg/detail/keypoints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Keypoints & msg,
  std::ostream & out)
{
  out << "{";
  // member: keypoints_1
  {
    if (msg.keypoints_1.size() == 0) {
      out << "keypoints_1: []";
    } else {
      out << "keypoints_1: [";
      size_t pending_items = msg.keypoints_1.size();
      for (auto item : msg.keypoints_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: keypoints_2
  {
    if (msg.keypoints_2.size() == 0) {
      out << "keypoints_2: []";
    } else {
      out << "keypoints_2: [";
      size_t pending_items = msg.keypoints_2.size();
      for (auto item : msg.keypoints_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Keypoints & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: keypoints_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.keypoints_1.size() == 0) {
      out << "keypoints_1: []\n";
    } else {
      out << "keypoints_1:\n";
      for (auto item : msg.keypoints_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: keypoints_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.keypoints_2.size() == 0) {
      out << "keypoints_2: []\n";
    } else {
      out << "keypoints_2:\n";
      for (auto item : msg.keypoints_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Keypoints & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace turtlebot_control

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot_control::msg::Keypoints & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot_control::msg::Keypoints & msg)
{
  return turtlebot_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot_control::msg::Keypoints>()
{
  return "turtlebot_control::msg::Keypoints";
}

template<>
inline const char * name<turtlebot_control::msg::Keypoints>()
{
  return "turtlebot_control/msg/Keypoints";
}

template<>
struct has_fixed_size<turtlebot_control::msg::Keypoints>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot_control::msg::Keypoints>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot_control::msg::Keypoints>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__TRAITS_HPP_
