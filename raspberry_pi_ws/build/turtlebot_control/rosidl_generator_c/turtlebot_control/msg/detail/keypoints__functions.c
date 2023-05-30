// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice
#include "turtlebot_control/msg/detail/keypoints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `keypoints_1`
// Member `keypoints_2`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
turtlebot_control__msg__Keypoints__init(turtlebot_control__msg__Keypoints * msg)
{
  if (!msg) {
    return false;
  }
  // keypoints_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->keypoints_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // keypoints_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->keypoints_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot_control__msg__Keypoints__fini(turtlebot_control__msg__Keypoints * msg)
{
  if (!msg) {
    return;
  }
  // keypoints_1
  rosidl_runtime_c__double__Sequence__fini(&msg->keypoints_1);
  // keypoints_2
  rosidl_runtime_c__double__Sequence__fini(&msg->keypoints_2);
}

bool
turtlebot_control__msg__Keypoints__are_equal(const turtlebot_control__msg__Keypoints * lhs, const turtlebot_control__msg__Keypoints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // keypoints_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->keypoints_1), &(rhs->keypoints_1)))
  {
    return false;
  }
  // keypoints_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->keypoints_2), &(rhs->keypoints_2)))
  {
    return false;
  }
  return true;
}

bool
turtlebot_control__msg__Keypoints__copy(
  const turtlebot_control__msg__Keypoints * input,
  turtlebot_control__msg__Keypoints * output)
{
  if (!input || !output) {
    return false;
  }
  // keypoints_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->keypoints_1), &(output->keypoints_1)))
  {
    return false;
  }
  // keypoints_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->keypoints_2), &(output->keypoints_2)))
  {
    return false;
  }
  return true;
}

turtlebot_control__msg__Keypoints *
turtlebot_control__msg__Keypoints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot_control__msg__Keypoints * msg = (turtlebot_control__msg__Keypoints *)allocator.allocate(sizeof(turtlebot_control__msg__Keypoints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot_control__msg__Keypoints));
  bool success = turtlebot_control__msg__Keypoints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot_control__msg__Keypoints__destroy(turtlebot_control__msg__Keypoints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot_control__msg__Keypoints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot_control__msg__Keypoints__Sequence__init(turtlebot_control__msg__Keypoints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot_control__msg__Keypoints * data = NULL;

  if (size) {
    data = (turtlebot_control__msg__Keypoints *)allocator.zero_allocate(size, sizeof(turtlebot_control__msg__Keypoints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot_control__msg__Keypoints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot_control__msg__Keypoints__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot_control__msg__Keypoints__Sequence__fini(turtlebot_control__msg__Keypoints__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot_control__msg__Keypoints__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot_control__msg__Keypoints__Sequence *
turtlebot_control__msg__Keypoints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot_control__msg__Keypoints__Sequence * array = (turtlebot_control__msg__Keypoints__Sequence *)allocator.allocate(sizeof(turtlebot_control__msg__Keypoints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot_control__msg__Keypoints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot_control__msg__Keypoints__Sequence__destroy(turtlebot_control__msg__Keypoints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot_control__msg__Keypoints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot_control__msg__Keypoints__Sequence__are_equal(const turtlebot_control__msg__Keypoints__Sequence * lhs, const turtlebot_control__msg__Keypoints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot_control__msg__Keypoints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot_control__msg__Keypoints__Sequence__copy(
  const turtlebot_control__msg__Keypoints__Sequence * input,
  turtlebot_control__msg__Keypoints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot_control__msg__Keypoints);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot_control__msg__Keypoints * data =
      (turtlebot_control__msg__Keypoints *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot_control__msg__Keypoints__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot_control__msg__Keypoints__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot_control__msg__Keypoints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
