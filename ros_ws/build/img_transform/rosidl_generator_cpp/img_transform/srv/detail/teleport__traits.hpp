// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from img_transform:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__SRV__DETAIL__TELEPORT__TRAITS_HPP_
#define IMG_TRANSFORM__SRV__DETAIL__TELEPORT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "img_transform/srv/detail/teleport__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace img_transform
{

namespace srv
{

inline void to_flow_style_yaml(
  const Teleport_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Teleport_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Teleport_Request & msg, bool use_flow_style = false)
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

}  // namespace img_transform

namespace rosidl_generator_traits
{

[[deprecated("use img_transform::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const img_transform::srv::Teleport_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  img_transform::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use img_transform::srv::to_yaml() instead")]]
inline std::string to_yaml(const img_transform::srv::Teleport_Request & msg)
{
  return img_transform::srv::to_yaml(msg);
}

template<>
inline const char * data_type<img_transform::srv::Teleport_Request>()
{
  return "img_transform::srv::Teleport_Request";
}

template<>
inline const char * name<img_transform::srv::Teleport_Request>()
{
  return "img_transform/srv/Teleport_Request";
}

template<>
struct has_fixed_size<img_transform::srv::Teleport_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<img_transform::srv::Teleport_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<img_transform::srv::Teleport_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace img_transform
{

namespace srv
{

inline void to_flow_style_yaml(
  const Teleport_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Teleport_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Teleport_Response & msg, bool use_flow_style = false)
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

}  // namespace img_transform

namespace rosidl_generator_traits
{

[[deprecated("use img_transform::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const img_transform::srv::Teleport_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  img_transform::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use img_transform::srv::to_yaml() instead")]]
inline std::string to_yaml(const img_transform::srv::Teleport_Response & msg)
{
  return img_transform::srv::to_yaml(msg);
}

template<>
inline const char * data_type<img_transform::srv::Teleport_Response>()
{
  return "img_transform::srv::Teleport_Response";
}

template<>
inline const char * name<img_transform::srv::Teleport_Response>()
{
  return "img_transform/srv/Teleport_Response";
}

template<>
struct has_fixed_size<img_transform::srv::Teleport_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<img_transform::srv::Teleport_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<img_transform::srv::Teleport_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<img_transform::srv::Teleport>()
{
  return "img_transform::srv::Teleport";
}

template<>
inline const char * name<img_transform::srv::Teleport>()
{
  return "img_transform/srv/Teleport";
}

template<>
struct has_fixed_size<img_transform::srv::Teleport>
  : std::integral_constant<
    bool,
    has_fixed_size<img_transform::srv::Teleport_Request>::value &&
    has_fixed_size<img_transform::srv::Teleport_Response>::value
  >
{
};

template<>
struct has_bounded_size<img_transform::srv::Teleport>
  : std::integral_constant<
    bool,
    has_bounded_size<img_transform::srv::Teleport_Request>::value &&
    has_bounded_size<img_transform::srv::Teleport_Response>::value
  >
{
};

template<>
struct is_service<img_transform::srv::Teleport>
  : std::true_type
{
};

template<>
struct is_service_request<img_transform::srv::Teleport_Request>
  : std::true_type
{
};

template<>
struct is_service_response<img_transform::srv::Teleport_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // IMG_TRANSFORM__SRV__DETAIL__TELEPORT__TRAITS_HPP_
