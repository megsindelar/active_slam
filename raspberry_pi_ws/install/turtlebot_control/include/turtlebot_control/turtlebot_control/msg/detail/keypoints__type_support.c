// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtlebot_control/msg/detail/keypoints__rosidl_typesupport_introspection_c.h"
#include "turtlebot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtlebot_control/msg/detail/keypoints__functions.h"
#include "turtlebot_control/msg/detail/keypoints__struct.h"


// Include directives for member types
// Member `x_1`
// Member `y_1`
// Member `size_1`
// Member `angle_1`
// Member `response_1`
// Member `octave_1`
// Member `class_id_1`
// Member `x_2`
// Member `y_2`
// Member `size_2`
// Member `angle_2`
// Member `response_2`
// Member `octave_2`
// Member `class_id_2`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot_control__msg__Keypoints__init(message_memory);
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_fini_function(void * message_memory)
{
  turtlebot_control__msg__Keypoints__fini(message_memory);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__x_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__x_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__x_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__x_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__y_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__y_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__y_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__y_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__size_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__size_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__size_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__size_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__angle_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__angle_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__angle_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__angle_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__response_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__response_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__response_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__response_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__octave_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__octave_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_1(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__octave_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_1(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__octave_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__class_id_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__class_id_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_1(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__class_id_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_1(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__class_id_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__x_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__x_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__x_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__x_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__y_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__y_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__y_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__y_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__size_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__size_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__size_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__size_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__angle_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__angle_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__angle_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__angle_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__response_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__response_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__response_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__response_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__octave_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__octave_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_2(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__octave_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_2(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__octave_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__class_id_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__class_id_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_2(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__class_id_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_2(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__class_id_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_member_array[14] = {
  {
    "x_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, x_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__x_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__x_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__x_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__x_1  // resize(index) function pointer
  },
  {
    "y_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, y_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__y_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__y_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__y_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__y_1  // resize(index) function pointer
  },
  {
    "size_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, size_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__size_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__size_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__size_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__size_1  // resize(index) function pointer
  },
  {
    "angle_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, angle_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__angle_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__angle_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__angle_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__angle_1  // resize(index) function pointer
  },
  {
    "response_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, response_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__response_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__response_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__response_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__response_1  // resize(index) function pointer
  },
  {
    "octave_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, octave_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__octave_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__octave_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__octave_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__octave_1  // resize(index) function pointer
  },
  {
    "class_id_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, class_id_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__class_id_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__class_id_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__class_id_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__class_id_1  // resize(index) function pointer
  },
  {
    "x_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, x_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__x_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__x_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__x_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__x_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__x_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__x_2  // resize(index) function pointer
  },
  {
    "y_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, y_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__y_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__y_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__y_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__y_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__y_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__y_2  // resize(index) function pointer
  },
  {
    "size_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, size_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__size_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__size_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__size_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__size_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__size_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__size_2  // resize(index) function pointer
  },
  {
    "angle_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, angle_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__angle_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__angle_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__angle_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__angle_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__angle_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__angle_2  // resize(index) function pointer
  },
  {
    "response_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, response_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__response_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__response_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__response_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__response_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__response_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__response_2  // resize(index) function pointer
  },
  {
    "octave_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, octave_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__octave_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__octave_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__octave_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__octave_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__octave_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__octave_2  // resize(index) function pointer
  },
  {
    "class_id_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, class_id_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__class_id_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__class_id_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__class_id_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__class_id_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__class_id_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__class_id_2  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_members = {
  "turtlebot_control__msg",  // message namespace
  "Keypoints",  // message name
  14,  // number of fields
  sizeof(turtlebot_control__msg__Keypoints),
  turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_member_array,  // message members
  turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_type_support_handle = {
  0,
  &turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot_control, msg, Keypoints)() {
  if (!turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_type_support_handle.typesupport_identifier) {
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
