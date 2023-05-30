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
  using _keypoints_1_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _keypoints_1_type keypoints_1;
  using _keypoints_2_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _keypoints_2_type keypoints_2;

  // setters for named parameter idiom
  Type & set__keypoints_1(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->keypoints_1 = _arg;
    return *this;
  }
  Type & set__keypoints_2(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->keypoints_2 = _arg;
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
    if (this->keypoints_1 != other.keypoints_1) {
      return false;
    }
    if (this->keypoints_2 != other.keypoints_2) {
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
