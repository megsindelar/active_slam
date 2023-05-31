// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_
#define TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot_control__msg__Keypoints __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot_control__msg__Keypoints __declspec(deprecated)
#endif

namespace turtlebot_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Keypoints_
{
  using Type = Keypoints_<ContainerAllocator>;

  explicit Keypoints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Keypoints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _x_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_1_type x_1;
  using _y_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_1_type y_1;
  using _size_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _size_1_type size_1;
  using _angle_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angle_1_type angle_1;
  using _response_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _response_1_type response_1;
  using _octave_1_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _octave_1_type octave_1;
  using _class_id_1_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _class_id_1_type class_id_1;
  using _x_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_2_type x_2;
  using _y_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_2_type y_2;
  using _size_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _size_2_type size_2;
  using _angle_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angle_2_type angle_2;
  using _response_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _response_2_type response_2;
  using _octave_2_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _octave_2_type octave_2;
  using _class_id_2_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _class_id_2_type class_id_2;

  // setters for named parameter idiom
  Type & set__x_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x_1 = _arg;
    return *this;
  }
  Type & set__y_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y_1 = _arg;
    return *this;
  }
  Type & set__size_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->size_1 = _arg;
    return *this;
  }
  Type & set__angle_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angle_1 = _arg;
    return *this;
  }
  Type & set__response_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->response_1 = _arg;
    return *this;
  }
  Type & set__octave_1(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->octave_1 = _arg;
    return *this;
  }
  Type & set__class_id_1(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->class_id_1 = _arg;
    return *this;
  }
  Type & set__x_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x_2 = _arg;
    return *this;
  }
  Type & set__y_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y_2 = _arg;
    return *this;
  }
  Type & set__size_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->size_2 = _arg;
    return *this;
  }
  Type & set__angle_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angle_2 = _arg;
    return *this;
  }
  Type & set__response_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->response_2 = _arg;
    return *this;
  }
  Type & set__octave_2(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->octave_2 = _arg;
    return *this;
  }
  Type & set__class_id_2(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->class_id_2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot_control::msg::Keypoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot_control::msg::Keypoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot_control::msg::Keypoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot_control::msg::Keypoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot_control__msg__Keypoints
    std::shared_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot_control__msg__Keypoints
    std::shared_ptr<turtlebot_control::msg::Keypoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Keypoints_ & other) const
  {
    if (this->x_1 != other.x_1) {
      return false;
    }
    if (this->y_1 != other.y_1) {
      return false;
    }
    if (this->size_1 != other.size_1) {
      return false;
    }
    if (this->angle_1 != other.angle_1) {
      return false;
    }
    if (this->response_1 != other.response_1) {
      return false;
    }
    if (this->octave_1 != other.octave_1) {
      return false;
    }
    if (this->class_id_1 != other.class_id_1) {
      return false;
    }
    if (this->x_2 != other.x_2) {
      return false;
    }
    if (this->y_2 != other.y_2) {
      return false;
    }
    if (this->size_2 != other.size_2) {
      return false;
    }
    if (this->angle_2 != other.angle_2) {
      return false;
    }
    if (this->response_2 != other.response_2) {
      return false;
    }
    if (this->octave_2 != other.octave_2) {
      return false;
    }
    if (this->class_id_2 != other.class_id_2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Keypoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Keypoints_

// alias to use template instance with default allocator
using Keypoints =
  turtlebot_control::msg::Keypoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtlebot_control

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_
