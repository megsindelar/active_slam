// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from img_transform:msg/Keypoints.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "img_transform/msg/detail/keypoints__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace img_transform
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Keypoints_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) img_transform::msg::Keypoints(_init);
}

void Keypoints_fini_function(void * message_memory)
{
  auto typed_message = static_cast<img_transform::msg::Keypoints *>(message_memory);
  typed_message->~Keypoints();
}

size_t size_function__Keypoints__x_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__x_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__x_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__x_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__x_1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__x_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__x_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__x_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__y_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__y_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__y_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__y_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__y_1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__y_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__y_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__y_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__size_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__size_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__size_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__size_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__size_1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__size_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__size_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__size_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__angle_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__angle_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__angle_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__angle_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__angle_1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__angle_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__angle_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__angle_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__response_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__response_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__response_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__response_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__response_1(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__response_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__response_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__response_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__octave_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__octave_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__octave_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__octave_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__Keypoints__octave_1(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__octave_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__Keypoints__octave_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__octave_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__class_id_1(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__class_id_1(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__class_id_1(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__class_id_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__Keypoints__class_id_1(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__class_id_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__Keypoints__class_id_1(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__class_id_1(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__x_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__x_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__x_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__x_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__x_2(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__x_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__x_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__x_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__y_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__y_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__y_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__y_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__y_2(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__y_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__y_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__y_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__size_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__size_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__size_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__size_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__size_2(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__size_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__size_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__size_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__angle_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__angle_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__angle_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__angle_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__angle_2(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__angle_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__angle_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__angle_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__response_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__response_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__response_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__response_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Keypoints__response_2(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__response_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Keypoints__response_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__response_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__octave_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__octave_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__octave_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__octave_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__Keypoints__octave_2(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__octave_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__Keypoints__octave_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__octave_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Keypoints__class_id_2(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keypoints__class_id_2(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Keypoints__class_id_2(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__Keypoints__class_id_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__Keypoints__class_id_2(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__Keypoints__class_id_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__Keypoints__class_id_2(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__Keypoints__class_id_2(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Keypoints_message_member_array[14] = {
  {
    "x_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, x_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__x_1,  // size() function pointer
    get_const_function__Keypoints__x_1,  // get_const(index) function pointer
    get_function__Keypoints__x_1,  // get(index) function pointer
    fetch_function__Keypoints__x_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__x_1,  // assign(index, value) function pointer
    resize_function__Keypoints__x_1  // resize(index) function pointer
  },
  {
    "y_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, y_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__y_1,  // size() function pointer
    get_const_function__Keypoints__y_1,  // get_const(index) function pointer
    get_function__Keypoints__y_1,  // get(index) function pointer
    fetch_function__Keypoints__y_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__y_1,  // assign(index, value) function pointer
    resize_function__Keypoints__y_1  // resize(index) function pointer
  },
  {
    "size_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, size_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__size_1,  // size() function pointer
    get_const_function__Keypoints__size_1,  // get_const(index) function pointer
    get_function__Keypoints__size_1,  // get(index) function pointer
    fetch_function__Keypoints__size_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__size_1,  // assign(index, value) function pointer
    resize_function__Keypoints__size_1  // resize(index) function pointer
  },
  {
    "angle_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, angle_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__angle_1,  // size() function pointer
    get_const_function__Keypoints__angle_1,  // get_const(index) function pointer
    get_function__Keypoints__angle_1,  // get(index) function pointer
    fetch_function__Keypoints__angle_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__angle_1,  // assign(index, value) function pointer
    resize_function__Keypoints__angle_1  // resize(index) function pointer
  },
  {
    "response_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, response_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__response_1,  // size() function pointer
    get_const_function__Keypoints__response_1,  // get_const(index) function pointer
    get_function__Keypoints__response_1,  // get(index) function pointer
    fetch_function__Keypoints__response_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__response_1,  // assign(index, value) function pointer
    resize_function__Keypoints__response_1  // resize(index) function pointer
  },
  {
    "octave_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, octave_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__octave_1,  // size() function pointer
    get_const_function__Keypoints__octave_1,  // get_const(index) function pointer
    get_function__Keypoints__octave_1,  // get(index) function pointer
    fetch_function__Keypoints__octave_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__octave_1,  // assign(index, value) function pointer
    resize_function__Keypoints__octave_1  // resize(index) function pointer
  },
  {
    "class_id_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, class_id_1),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__class_id_1,  // size() function pointer
    get_const_function__Keypoints__class_id_1,  // get_const(index) function pointer
    get_function__Keypoints__class_id_1,  // get(index) function pointer
    fetch_function__Keypoints__class_id_1,  // fetch(index, &value) function pointer
    assign_function__Keypoints__class_id_1,  // assign(index, value) function pointer
    resize_function__Keypoints__class_id_1  // resize(index) function pointer
  },
  {
    "x_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, x_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__x_2,  // size() function pointer
    get_const_function__Keypoints__x_2,  // get_const(index) function pointer
    get_function__Keypoints__x_2,  // get(index) function pointer
    fetch_function__Keypoints__x_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__x_2,  // assign(index, value) function pointer
    resize_function__Keypoints__x_2  // resize(index) function pointer
  },
  {
    "y_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, y_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__y_2,  // size() function pointer
    get_const_function__Keypoints__y_2,  // get_const(index) function pointer
    get_function__Keypoints__y_2,  // get(index) function pointer
    fetch_function__Keypoints__y_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__y_2,  // assign(index, value) function pointer
    resize_function__Keypoints__y_2  // resize(index) function pointer
  },
  {
    "size_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, size_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__size_2,  // size() function pointer
    get_const_function__Keypoints__size_2,  // get_const(index) function pointer
    get_function__Keypoints__size_2,  // get(index) function pointer
    fetch_function__Keypoints__size_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__size_2,  // assign(index, value) function pointer
    resize_function__Keypoints__size_2  // resize(index) function pointer
  },
  {
    "angle_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, angle_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__angle_2,  // size() function pointer
    get_const_function__Keypoints__angle_2,  // get_const(index) function pointer
    get_function__Keypoints__angle_2,  // get(index) function pointer
    fetch_function__Keypoints__angle_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__angle_2,  // assign(index, value) function pointer
    resize_function__Keypoints__angle_2  // resize(index) function pointer
  },
  {
    "response_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, response_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__response_2,  // size() function pointer
    get_const_function__Keypoints__response_2,  // get_const(index) function pointer
    get_function__Keypoints__response_2,  // get(index) function pointer
    fetch_function__Keypoints__response_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__response_2,  // assign(index, value) function pointer
    resize_function__Keypoints__response_2  // resize(index) function pointer
  },
  {
    "octave_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, octave_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__octave_2,  // size() function pointer
    get_const_function__Keypoints__octave_2,  // get_const(index) function pointer
    get_function__Keypoints__octave_2,  // get(index) function pointer
    fetch_function__Keypoints__octave_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__octave_2,  // assign(index, value) function pointer
    resize_function__Keypoints__octave_2  // resize(index) function pointer
  },
  {
    "class_id_2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(img_transform::msg::Keypoints, class_id_2),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keypoints__class_id_2,  // size() function pointer
    get_const_function__Keypoints__class_id_2,  // get_const(index) function pointer
    get_function__Keypoints__class_id_2,  // get(index) function pointer
    fetch_function__Keypoints__class_id_2,  // fetch(index, &value) function pointer
    assign_function__Keypoints__class_id_2,  // assign(index, value) function pointer
    resize_function__Keypoints__class_id_2  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Keypoints_message_members = {
  "img_transform::msg",  // message namespace
  "Keypoints",  // message name
  14,  // number of fields
  sizeof(img_transform::msg::Keypoints),
  Keypoints_message_member_array,  // message members
  Keypoints_init_function,  // function to initialize message memory (memory has to be allocated)
  Keypoints_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Keypoints_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Keypoints_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace img_transform


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<img_transform::msg::Keypoints>()
{
  return &::img_transform::msg::rosidl_typesupport_introspection_cpp::Keypoints_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, img_transform, msg, Keypoints)() {
  return &::img_transform::msg::rosidl_typesupport_introspection_cpp::Keypoints_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
