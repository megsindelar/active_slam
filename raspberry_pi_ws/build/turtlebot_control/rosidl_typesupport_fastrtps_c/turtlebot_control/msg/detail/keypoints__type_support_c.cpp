// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice
#include "turtlebot_control/msg/detail/keypoints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "turtlebot_control/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtlebot_control/msg/detail/keypoints__struct.h"
#include "turtlebot_control/msg/detail/keypoints__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // keypoints_1, keypoints_2
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // keypoints_1, keypoints_2

// forward declare type support functions


using _Keypoints__ros_msg_type = turtlebot_control__msg__Keypoints;

static bool _Keypoints__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Keypoints__ros_msg_type * ros_message = static_cast<const _Keypoints__ros_msg_type *>(untyped_ros_message);
  // Field name: keypoints_1
  {
    size_t size = ros_message->keypoints_1.size;
    auto array_ptr = ros_message->keypoints_1.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: keypoints_2
  {
    size_t size = ros_message->keypoints_2.size;
    auto array_ptr = ros_message->keypoints_2.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Keypoints__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Keypoints__ros_msg_type * ros_message = static_cast<_Keypoints__ros_msg_type *>(untyped_ros_message);
  // Field name: keypoints_1
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->keypoints_1.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->keypoints_1);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->keypoints_1, size)) {
      fprintf(stderr, "failed to create array for field 'keypoints_1'");
      return false;
    }
    auto array_ptr = ros_message->keypoints_1.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: keypoints_2
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->keypoints_2.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->keypoints_2);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->keypoints_2, size)) {
      fprintf(stderr, "failed to create array for field 'keypoints_2'");
      return false;
    }
    auto array_ptr = ros_message->keypoints_2.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot_control
size_t get_serialized_size_turtlebot_control__msg__Keypoints(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Keypoints__ros_msg_type * ros_message = static_cast<const _Keypoints__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name keypoints_1
  {
    size_t array_size = ros_message->keypoints_1.size;
    auto array_ptr = ros_message->keypoints_1.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name keypoints_2
  {
    size_t array_size = ros_message->keypoints_2.size;
    auto array_ptr = ros_message->keypoints_2.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Keypoints__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtlebot_control__msg__Keypoints(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtlebot_control
size_t max_serialized_size_turtlebot_control__msg__Keypoints(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: keypoints_1
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: keypoints_2
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Keypoints__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_turtlebot_control__msg__Keypoints(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Keypoints = {
  "turtlebot_control::msg",
  "Keypoints",
  _Keypoints__cdr_serialize,
  _Keypoints__cdr_deserialize,
  _Keypoints__get_serialized_size,
  _Keypoints__max_serialized_size
};

static rosidl_message_type_support_t _Keypoints__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Keypoints,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot_control, msg, Keypoints)() {
  return &_Keypoints__type_support;
}

#if defined(__cplusplus)
}
#endif
