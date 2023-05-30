// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_H_
#define TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'keypoints_1'
// Member 'keypoints_2'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Keypoints in the package turtlebot_control.
typedef struct turtlebot_control__msg__Keypoints
{
  rosidl_runtime_c__double__Sequence keypoints_1;
  rosidl_runtime_c__double__Sequence keypoints_2;
} turtlebot_control__msg__Keypoints;

// Struct for a sequence of turtlebot_control__msg__Keypoints.
typedef struct turtlebot_control__msg__Keypoints__Sequence
{
  turtlebot_control__msg__Keypoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot_control__msg__Keypoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_H_
