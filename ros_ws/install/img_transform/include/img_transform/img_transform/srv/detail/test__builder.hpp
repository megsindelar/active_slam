// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from img_transform:srv/Test.idl
// generated code does not contain a copyright notice

#ifndef IMG_TRANSFORM__SRV__DETAIL__TEST__BUILDER_HPP_
#define IMG_TRANSFORM__SRV__DETAIL__TEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "img_transform/srv/detail/test__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace img_transform
{

namespace srv
{

namespace builder
{

class Init_Test_Request_n
{
public:
  Init_Test_Request_n()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::img_transform::srv::Test_Request n(::img_transform::srv::Test_Request::_n_type arg)
  {
    msg_.n = std::move(arg);
    return std::move(msg_);
  }

private:
  ::img_transform::srv::Test_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::img_transform::srv::Test_Request>()
{
  return img_transform::srv::builder::Init_Test_Request_n();
}

}  // namespace img_transform


namespace img_transform
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::img_transform::srv::Test_Response>()
{
  return ::img_transform::srv::Test_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace img_transform

#endif  // IMG_TRANSFORM__SRV__DETAIL__TEST__BUILDER_HPP_
