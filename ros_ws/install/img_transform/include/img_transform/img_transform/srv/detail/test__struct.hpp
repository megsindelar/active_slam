// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from img_transform:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__SRV__DETAIL__TEST__STRUCT_HPP_
#define IMG_TRANSFORM__SRV__DETAIL__TEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__img_transform__srv__Test_Request __attribute__((deprecated))
#else
# define DEPRECATED__img_transform__srv__Test_Request __declspec(deprecated)
#endif

namespace img_transform
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Test_Request_
{
  using Type = Test_Request_<ContainerAllocator>;

  explicit Test_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0.0;
    }
  }

  explicit Test_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0.0;
    }
  }

  // field types and members
  using _n_type =
    double;
  _n_type n;

  // setters for named parameter idiom
  Type & set__n(
    const double & _arg)
  {
    this->n = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    img_transform::srv::Test_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const img_transform::srv::Test_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<img_transform::srv::Test_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<img_transform::srv::Test_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      img_transform::srv::Test_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<img_transform::srv::Test_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      img_transform::srv::Test_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<img_transform::srv::Test_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<img_transform::srv::Test_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<img_transform::srv::Test_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__img_transform__srv__Test_Request
    std::shared_ptr<img_transform::srv::Test_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__img_transform__srv__Test_Request
    std::shared_ptr<img_transform::srv::Test_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Test_Request_ & other) const
  {
    if (this->n != other.n) {
      return false;
    }
    return true;
  }
  bool operator!=(const Test_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Test_Request_

// alias to use template instance with default allocator
using Test_Request =
  img_transform::srv::Test_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace img_transform


#ifndef _WIN32
# define DEPRECATED__img_transform__srv__Test_Response __attribute__((deprecated))
#else
# define DEPRECATED__img_transform__srv__Test_Response __declspec(deprecated)
#endif

namespace img_transform
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Test_Response_
{
  using Type = Test_Response_<ContainerAllocator>;

  explicit Test_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Test_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    img_transform::srv::Test_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const img_transform::srv::Test_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<img_transform::srv::Test_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<img_transform::srv::Test_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      img_transform::srv::Test_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<img_transform::srv::Test_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      img_transform::srv::Test_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<img_transform::srv::Test_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<img_transform::srv::Test_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<img_transform::srv::Test_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__img_transform__srv__Test_Response
    std::shared_ptr<img_transform::srv::Test_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__img_transform__srv__Test_Response
    std::shared_ptr<img_transform::srv::Test_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Test_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Test_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Test_Response_

// alias to use template instance with default allocator
using Test_Response =
  img_transform::srv::Test_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace img_transform

namespace img_transform
{

namespace srv
{

struct Test
{
  using Request = img_transform::srv::Test_Request;
  using Response = img_transform::srv::Test_Response;
};

}  // namespace srv

}  // namespace img_transform

#endif  // IMG_TRANSFORM__SRV__DETAIL__TEST__STRUCT_HPP_
