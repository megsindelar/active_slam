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
// Member `keypoints_1`
// Member `keypoints_2`
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

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__keypoints_1(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_1(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_1(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__keypoints_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_1(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__keypoints_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_1(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__keypoints_1(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__keypoints_2(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_2(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_2(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__keypoints_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_2(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__keypoints_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_2(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__keypoints_2(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_member_array[2] = {
  {
    "keypoints_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, keypoints_1),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__keypoints_1,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_1,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_1,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__keypoints_1,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__keypoints_1,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__keypoints_1  // resize(index) function pointer
  },
  {
    "keypoints_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot_control__msg__Keypoints, keypoints_2),  // bytes offset in struct
    NULL,  // default value
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__size_function__Keypoints__keypoints_2,  // size() function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_const_function__Keypoints__keypoints_2,  // get_const(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__get_function__Keypoints__keypoints_2,  // get(index) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__fetch_function__Keypoints__keypoints_2,  // fetch(index, &value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__assign_function__Keypoints__keypoints_2,  // assign(index, value) function pointer
    turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__resize_function__Keypoints__keypoints_2  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot_control__msg__Keypoints__rosidl_typesupport_introspection_c__Keypoints_message_members = {
  "turtlebot_control__msg",  // message namespace
  "Keypoints",  // message name
  2,  // number of fields
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
