// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from img_transform:srv/Test.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "img_transform/srv/detail/test__rosidl_typesupport_introspection_c.h"
#include "img_transform/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "img_transform/srv/detail/test__functions.h"
#include "img_transform/srv/detail/test__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  img_transform__srv__Test_Request__init(message_memory);
}

void img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_fini_function(void * message_memory)
{
  img_transform__srv__Test_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_member_array[1] = {
  {
    "n",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform__srv__Test_Request, n),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_members = {
  "img_transform__srv",  // message namespace
  "Test_Request",  // message name
  1,  // number of fields
  sizeof(img_transform__srv__Test_Request),
  img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_member_array,  // message members
  img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_type_support_handle = {
  0,
  &img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_img_transform
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Request)() {
  if (!img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_type_support_handle.typesupport_identifier) {
    img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &img_transform__srv__Test_Request__rosidl_typesupport_introspection_c__Test_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "img_transform/srv/detail/test__rosidl_typesupport_introspection_c.h"
// already included above
// #include "img_transform/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "img_transform/srv/detail/test__functions.h"
// already included above
// #include "img_transform/srv/detail/test__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  img_transform__srv__Test_Response__init(message_memory);
}

void img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_fini_function(void * message_memory)
{
  img_transform__srv__Test_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform__srv__Test_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_members = {
  "img_transform__srv",  // message namespace
  "Test_Response",  // message name
  1,  // number of fields
  sizeof(img_transform__srv__Test_Response),
  img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_member_array,  // message members
  img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_type_support_handle = {
  0,
  &img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_img_transform
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Response)() {
  if (!img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_type_support_handle.typesupport_identifier) {
    img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &img_transform__srv__Test_Response__rosidl_typesupport_introspection_c__Test_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "img_transform/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "img_transform/srv/detail/test__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_members = {
  "img_transform__srv",  // service namespace
  "Test",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_Request_message_type_support_handle,
  NULL  // response message
  // img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_Response_message_type_support_handle
};

static rosidl_service_type_support_t img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_type_support_handle = {
  0,
  &img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_img_transform
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test)() {
  if (!img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_type_support_handle.typesupport_identifier) {
    img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, img_transform, srv, Test_Response)()->data;
  }

  return &img_transform__srv__detail__test__rosidl_typesupport_introspection_c__Test_service_type_support_handle;
}
