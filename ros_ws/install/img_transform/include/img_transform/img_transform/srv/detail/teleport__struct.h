// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from img_transform:srv/Teleport.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__SRV__DETAIL__TELEPORT__STRUCT_H_
#define IMG_TRANSFORM__SRV__DETAIL__TELEPORT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Teleport in the package img_transform.
typedef struct img_transform__srv__Teleport_Request
{
  double x;
  double y;
  double theta;
} img_transform__srv__Teleport_Request;

// Struct for a sequence of img_transform__srv__Teleport_Request.
typedef struct img_transform__srv__Teleport_Request__Sequence
{
  img_transform__srv__Teleport_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} img_transform__srv__Teleport_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Teleport in the package img_transform.
typedef struct img_transform__srv__Teleport_Response
{
  uint8_t structure_needs_at_least_one_member;
} img_transform__srv__Teleport_Response;

// Struct for a sequence of img_transform__srv__Teleport_Response.
typedef struct img_transform__srv__Teleport_Response__Sequence
{
  img_transform__srv__Teleport_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} img_transform__srv__Teleport_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IMG_TRANSFORM__SRV__DETAIL__TELEPORT__STRUCT_H_
