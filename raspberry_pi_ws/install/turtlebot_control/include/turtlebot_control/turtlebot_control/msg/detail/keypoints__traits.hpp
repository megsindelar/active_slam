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
  // member: x_1
  {
    if (msg.x_1.size() == 0) {
      out << "x_1: []";
    } else {
      out << "x_1: [";
      size_t pending_items = msg.x_1.size();
      for (auto item : msg.x_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y_1
  {
    if (msg.y_1.size() == 0) {
      out << "y_1: []";
    } else {
      out << "y_1: [";
      size_t pending_items = msg.y_1.size();
      for (auto item : msg.y_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: size_1
  {
    if (msg.size_1.size() == 0) {
      out << "size_1: []";
    } else {
      out << "size_1: [";
      size_t pending_items = msg.size_1.size();
      for (auto item : msg.size_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: angle_1
  {
    if (msg.angle_1.size() == 0) {
      out << "angle_1: []";
    } else {
      out << "angle_1: [";
      size_t pending_items = msg.angle_1.size();
      for (auto item : msg.angle_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response_1
  {
    if (msg.response_1.size() == 0) {
      out << "response_1: []";
    } else {
      out << "response_1: [";
      size_t pending_items = msg.response_1.size();
      for (auto item : msg.response_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: octave_1
  {
    if (msg.octave_1.size() == 0) {
      out << "octave_1: []";
    } else {
      out << "octave_1: [";
      size_t pending_items = msg.octave_1.size();
      for (auto item : msg.octave_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: class_id_1
  {
    if (msg.class_id_1.size() == 0) {
      out << "class_id_1: []";
    } else {
      out << "class_id_1: [";
      size_t pending_items = msg.class_id_1.size();
      for (auto item : msg.class_id_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: x_2
  {
    if (msg.x_2.size() == 0) {
      out << "x_2: []";
    } else {
      out << "x_2: [";
      size_t pending_items = msg.x_2.size();
      for (auto item : msg.x_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y_2
  {
    if (msg.y_2.size() == 0) {
      out << "y_2: []";
    } else {
      out << "y_2: [";
      size_t pending_items = msg.y_2.size();
      for (auto item : msg.y_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: size_2
  {
    if (msg.size_2.size() == 0) {
      out << "size_2: []";
    } else {
      out << "size_2: [";
      size_t pending_items = msg.size_2.size();
      for (auto item : msg.size_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: angle_2
  {
    if (msg.angle_2.size() == 0) {
      out << "angle_2: []";
    } else {
      out << "angle_2: [";
      size_t pending_items = msg.angle_2.size();
      for (auto item : msg.angle_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response_2
  {
    if (msg.response_2.size() == 0) {
      out << "response_2: []";
    } else {
      out << "response_2: [";
      size_t pending_items = msg.response_2.size();
      for (auto item : msg.response_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: octave_2
  {
    if (msg.octave_2.size() == 0) {
      out << "octave_2: []";
    } else {
      out << "octave_2: [";
      size_t pending_items = msg.octave_2.size();
      for (auto item : msg.octave_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: class_id_2
  {
    if (msg.class_id_2.size() == 0) {
      out << "class_id_2: []";
    } else {
      out << "class_id_2: [";
      size_t pending_items = msg.class_id_2.size();
      for (auto item : msg.class_id_2) {
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
  // member: x_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x_1.size() == 0) {
      out << "x_1: []\n";
    } else {
      out << "x_1:\n";
      for (auto item : msg.x_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y_1.size() == 0) {
      out << "y_1: []\n";
    } else {
      out << "y_1:\n";
      for (auto item : msg.y_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: size_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.size_1.size() == 0) {
      out << "size_1: []\n";
    } else {
      out << "size_1:\n";
      for (auto item : msg.size_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: angle_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angle_1.size() == 0) {
      out << "angle_1: []\n";
    } else {
      out << "angle_1:\n";
      for (auto item : msg.angle_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: response_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response_1.size() == 0) {
      out << "response_1: []\n";
    } else {
      out << "response_1:\n";
      for (auto item : msg.response_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: octave_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.octave_1.size() == 0) {
      out << "octave_1: []\n";
    } else {
      out << "octave_1:\n";
      for (auto item : msg.octave_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: class_id_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.class_id_1.size() == 0) {
      out << "class_id_1: []\n";
    } else {
      out << "class_id_1:\n";
      for (auto item : msg.class_id_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: x_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x_2.size() == 0) {
      out << "x_2: []\n";
    } else {
      out << "x_2:\n";
      for (auto item : msg.x_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y_2.size() == 0) {
      out << "y_2: []\n";
    } else {
      out << "y_2:\n";
      for (auto item : msg.y_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: size_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.size_2.size() == 0) {
      out << "size_2: []\n";
    } else {
      out << "size_2:\n";
      for (auto item : msg.size_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: angle_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angle_2.size() == 0) {
      out << "angle_2: []\n";
    } else {
      out << "angle_2:\n";
      for (auto item : msg.angle_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: response_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response_2.size() == 0) {
      out << "response_2: []\n";
    } else {
      out << "response_2:\n";
      for (auto item : msg.response_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: octave_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.octave_2.size() == 0) {
      out << "octave_2: []\n";
    } else {
      out << "octave_2:\n";
      for (auto item : msg.octave_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: class_id_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.class_id_2.size() == 0) {
      out << "class_id_2: []\n";
    } else {
      out << "class_id_2:\n";
      for (auto item : msg.class_id_2) {
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
