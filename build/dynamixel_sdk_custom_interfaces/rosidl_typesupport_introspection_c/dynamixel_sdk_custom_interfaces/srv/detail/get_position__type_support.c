// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__rosidl_typesupport_introspection_c.h"
#include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__functions.h"
#include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__init(message_memory);
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_fini_function(void * message_memory)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_member_array[1] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_members = {
  "dynamixel_sdk_custom_interfaces__srv",  // message namespace
  "GetPosition_Request",  // message name
  1,  // number of fields
  sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Request),
  false,  // has_any_key_member_
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_member_array,  // message members
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle = {
  0,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_members,
  get_message_typesupport_handle_function,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__get_type_hash,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__get_type_description,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_sdk_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Request)() {
  if (!dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle.typesupport_identifier) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__functions.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__init(message_memory);
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_fini_function(void * message_memory)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_member_array[1] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_members = {
  "dynamixel_sdk_custom_interfaces__srv",  // message namespace
  "GetPosition_Response",  // message name
  1,  // number of fields
  sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Response),
  false,  // has_any_key_member_
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_member_array,  // message members
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle = {
  0,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_members,
  get_message_typesupport_handle_function,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__get_type_hash,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__get_type_description,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_sdk_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Response)() {
  if (!dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle.typesupport_identifier) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__functions.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "dynamixel_sdk_custom_interfaces/srv/get_position.h"
// Member `request`
// Member `response`
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__init(message_memory);
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_fini_function(void * message_memory)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__fini(message_memory);
}

size_t dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__size_function__GetPosition_Event__request(
  const void * untyped_member)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__request(
  const void * untyped_member, size_t index)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__request(
  void * untyped_member, size_t index)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * member =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__fetch_function__GetPosition_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * item =
    ((const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__request(untyped_member, index));
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * value =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)(untyped_value);
  *value = *item;
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__assign_function__GetPosition_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * item =
    ((dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__request(untyped_member, index));
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request * value =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Request *)(untyped_value);
  *item = *value;
}

bool dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__resize_function__GetPosition_Event__request(
  void * untyped_member, size_t size)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence * member =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence *)(untyped_member);
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__fini(member);
  return dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__Sequence__init(member, size);
}

size_t dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__size_function__GetPosition_Event__response(
  const void * untyped_member)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__response(
  const void * untyped_member, size_t index)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * member =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__response(
  void * untyped_member, size_t index)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * member =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__fetch_function__GetPosition_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * item =
    ((const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__response(untyped_member, index));
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * value =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)(untyped_value);
  *value = *item;
}

void dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__assign_function__GetPosition_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * item =
    ((dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__response(untyped_member, index));
  const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response * value =
    (const dynamixel_sdk_custom_interfaces__srv__GetPosition_Response *)(untyped_value);
  *item = *value;
}

bool dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__resize_function__GetPosition_Event__response(
  void * untyped_member, size_t size)
{
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence * member =
    (dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence *)(untyped_member);
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__fini(member);
  return dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Event, request),  // bytes offset in struct
    NULL,  // default value
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__size_function__GetPosition_Event__request,  // size() function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__request,  // get_const(index) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__request,  // get(index) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__fetch_function__GetPosition_Event__request,  // fetch(index, &value) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__assign_function__GetPosition_Event__request,  // assign(index, value) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__resize_function__GetPosition_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Event, response),  // bytes offset in struct
    NULL,  // default value
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__size_function__GetPosition_Event__response,  // size() function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_const_function__GetPosition_Event__response,  // get_const(index) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__get_function__GetPosition_Event__response,  // get(index) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__fetch_function__GetPosition_Event__response,  // fetch(index, &value) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__assign_function__GetPosition_Event__response,  // assign(index, value) function pointer
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__resize_function__GetPosition_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_members = {
  "dynamixel_sdk_custom_interfaces__srv",  // message namespace
  "GetPosition_Event",  // message name
  3,  // number of fields
  sizeof(dynamixel_sdk_custom_interfaces__srv__GetPosition_Event),
  false,  // has_any_key_member_
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_member_array,  // message members
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_type_support_handle = {
  0,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_members,
  get_message_typesupport_handle_function,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__get_type_hash,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__get_type_description,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_sdk_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Event)() {
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Request)();
  dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Response)();
  if (!dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_type_support_handle.typesupport_identifier) {
    dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_members = {
  "dynamixel_sdk_custom_interfaces__srv",  // service namespace
  "GetPosition",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle,
  NULL,  // response message
  // dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle
  NULL  // event_message
  // dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle
};


static rosidl_service_type_support_t dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_type_support_handle = {
  0,
  &dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_members,
  get_service_typesupport_handle_function,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Request__rosidl_typesupport_introspection_c__GetPosition_Request_message_type_support_handle,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Response__rosidl_typesupport_introspection_c__GetPosition_Response_message_type_support_handle,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition_Event__rosidl_typesupport_introspection_c__GetPosition_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dynamixel_sdk_custom_interfaces,
    srv,
    GetPosition
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dynamixel_sdk_custom_interfaces,
    srv,
    GetPosition
  ),
  &dynamixel_sdk_custom_interfaces__srv__GetPosition__get_type_hash,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition__get_type_description,
  &dynamixel_sdk_custom_interfaces__srv__GetPosition__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_sdk_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition)(void) {
  if (!dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_type_support_handle.typesupport_identifier) {
    dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_sdk_custom_interfaces, srv, GetPosition_Event)()->data;
  }

  return &dynamixel_sdk_custom_interfaces__srv__detail__get_position__rosidl_typesupport_introspection_c__GetPosition_service_type_support_handle;
}
