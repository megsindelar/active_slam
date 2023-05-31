// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from img_transform:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__MSG__DETAIL__KEYPOINTS__STRUCT_H_
#define IMG_TRANSFORM__MSG__DETAIL__KEYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x_1'
// Member 'y_1'
// Member 'size_1'
// Member 'angle_1'
// Member 'response_1'
// Member 'octave_1'
// Member 'class_id_1'
// Member 'x_2'
// Member 'y_2'
// Member 'size_2'
// Member 'angle_2'
// Member 'response_2'
// Member 'octave_2'
// Member 'class_id_2'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Keypoints in the package img_transform.
typedef struct img_transform__msg__Keypoints
{
  rosidl_runtime_c__double__Sequence x_1;
  rosidl_runtime_c__double__Sequence y_1;
  rosidl_runtime_c__double__Sequence size_1;
  rosidl_runtime_c__double__Sequence angle_1;
  rosidl_runtime_c__double__Sequence response_1;
  rosidl_runtime_c__int32__Sequence octave_1;
  rosidl_runtime_c__int32__Sequence class_id_1;
  rosidl_runtime_c__double__Sequence x_2;
  rosidl_runtime_c__double__Sequence y_2;
  rosidl_runtime_c__double__Sequence size_2;
  rosidl_runtime_c__double__Sequence angle_2;
  rosidl_runtime_c__double__Sequence response_2;
  rosidl_runtime_c__int32__Sequence octave_2;
  rosidl_runtime_c__int32__Sequence class_id_2;
} img_transform__msg__Keypoints;

// Struct for a sequence of img_transform__msg__Keypoints.
typedef struct img_transform__msg__Keypoints__Sequence
{
  img_transform__msg__Keypoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} img_transform__msg__Keypoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IMG_TRANSFORM__MSG__DETAIL__KEYPOINTS__STRUCT_H_
