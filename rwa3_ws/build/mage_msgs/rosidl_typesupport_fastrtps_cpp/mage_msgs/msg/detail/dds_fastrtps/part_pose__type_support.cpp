// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mage_msgs:msg/PartPose.idl
// generated code does not contain a copyright notice
#include "mage_msgs/msg/detail/part_pose__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mage_msgs/msg/detail/part_pose__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace mage_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const mage_msgs::msg::Part &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  mage_msgs::msg::Part &);
size_t get_serialized_size(
  const mage_msgs::msg::Part &,
  size_t current_alignment);
size_t
max_serialized_size_Part(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace mage_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Pose &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Pose &);
size_t get_serialized_size(
  const geometry_msgs::msg::Pose &,
  size_t current_alignment);
size_t
max_serialized_size_Pose(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace mage_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mage_msgs
cdr_serialize(
  const mage_msgs::msg::PartPose & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: part
  mage_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.part,
    cdr);
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mage_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mage_msgs::msg::PartPose & ros_message)
{
  // Member: part
  mage_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.part);

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mage_msgs
get_serialized_size(
  const mage_msgs::msg::PartPose & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: part

  current_alignment +=
    mage_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.part, current_alignment);
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mage_msgs
max_serialized_size_PartPose(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: part
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mage_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Part(
        full_bounded, current_alignment);
    }
  }

  // Member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _PartPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mage_msgs::msg::PartPose *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PartPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mage_msgs::msg::PartPose *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PartPose__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mage_msgs::msg::PartPose *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PartPose__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PartPose(full_bounded, 0);
}

static message_type_support_callbacks_t _PartPose__callbacks = {
  "mage_msgs::msg",
  "PartPose",
  _PartPose__cdr_serialize,
  _PartPose__cdr_deserialize,
  _PartPose__get_serialized_size,
  _PartPose__max_serialized_size
};

static rosidl_message_type_support_t _PartPose__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PartPose__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mage_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mage_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<mage_msgs::msg::PartPose>()
{
  return &mage_msgs::msg::typesupport_fastrtps_cpp::_PartPose__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mage_msgs, msg, PartPose)() {
  return &mage_msgs::msg::typesupport_fastrtps_cpp::_PartPose__handle;
}

#ifdef __cplusplus
}
#endif
