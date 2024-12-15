// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from object_detection_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "object_detection_msgs/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace object_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_Object_vel_y
{
public:
  explicit Init_Object_vel_y(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  ::object_detection_msgs::msg::Object vel_y(::object_detection_msgs::msg::Object::_vel_y_type arg)
  {
    msg_.vel_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_vel_x
{
public:
  explicit Init_Object_vel_x(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_vel_y vel_x(::object_detection_msgs::msg::Object::_vel_x_type arg)
  {
    msg_.vel_x = std::move(arg);
    return Init_Object_vel_y(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_width
{
public:
  explicit Init_Object_width(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_vel_x width(::object_detection_msgs::msg::Object::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Object_vel_x(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_height
{
public:
  explicit Init_Object_height(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_width height(::object_detection_msgs::msg::Object::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Object_width(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_y
{
public:
  explicit Init_Object_y(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_height y(::object_detection_msgs::msg::Object::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Object_height(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_x
{
public:
  explicit Init_Object_x(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_y x(::object_detection_msgs::msg::Object::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Object_y(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_confidence
{
public:
  explicit Init_Object_confidence(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_x confidence(::object_detection_msgs::msg::Object::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Object_x(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_label
{
public:
  explicit Init_Object_label(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_confidence label(::object_detection_msgs::msg::Object::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Object_confidence(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_id
{
public:
  explicit Init_Object_id(::object_detection_msgs::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_label id(::object_detection_msgs::msg::Object::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Object_label(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

class Init_Object_class_id
{
public:
  Init_Object_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_id class_id(::object_detection_msgs::msg::Object::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_Object_id(msg_);
  }

private:
  ::object_detection_msgs::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::object_detection_msgs::msg::Object>()
{
  return object_detection_msgs::msg::builder::Init_Object_class_id();
}

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__BUILDER_HPP_
