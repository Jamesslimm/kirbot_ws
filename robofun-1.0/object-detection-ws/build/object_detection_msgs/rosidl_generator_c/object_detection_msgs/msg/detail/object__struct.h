// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from object_detection_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Object in the package object_detection_msgs.
typedef struct object_detection_msgs__msg__Object
{
  /// class id
  int16_t class_id;
  /// track id
  int16_t id;
  /// object class
  rosidl_runtime_c__String label;
  /// confidence of result
  float confidence;
  /// normalized value of box top-left point coordinate x
  float x;
  /// normalized value of box top-left point coordinate y
  float y;
  /// normalized value of box height
  float height;
  /// normalized value of box width
  float width;
  /// normalized velocity of box top-left point coordinate x
  float vel_x;
  /// normalized velocity of box top-left point coordinate y
  float vel_y;
} object_detection_msgs__msg__Object;

// Struct for a sequence of object_detection_msgs__msg__Object.
typedef struct object_detection_msgs__msg__Object__Sequence
{
  object_detection_msgs__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} object_detection_msgs__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
