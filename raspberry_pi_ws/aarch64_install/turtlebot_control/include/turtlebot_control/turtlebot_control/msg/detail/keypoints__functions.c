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
// Member `x_1`
// Member `y_1`
// Member `size_1`
// Member `angle_1`
// Member `response_1`
// Member `octave_1`
// Member `class_id_1`
// Member `x_2`
// Member `y_2`
// Member `size_2`
// Member `angle_2`
// Member `response_2`
// Member `octave_2`
// Member `class_id_2`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
turtlebot_control__msg__Keypoints__init(turtlebot_control__msg__Keypoints * msg)
{
  if (!msg) {
    return false;
  }
  // x_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // y_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // size_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->size_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // angle_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->angle_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // response_1
  if (!rosidl_runtime_c__double__Sequence__init(&msg->response_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // octave_1
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->octave_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // class_id_1
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->class_id_1, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // x_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // y_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // size_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->size_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // angle_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->angle_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // response_2
  if (!rosidl_runtime_c__double__Sequence__init(&msg->response_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // octave_2
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->octave_2, 0)) {
    turtlebot_control__msg__Keypoints__fini(msg);
    return false;
  }
  // class_id_2
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->class_id_2, 0)) {
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
  // x_1
  rosidl_runtime_c__double__Sequence__fini(&msg->x_1);
  // y_1
  rosidl_runtime_c__double__Sequence__fini(&msg->y_1);
  // size_1
  rosidl_runtime_c__double__Sequence__fini(&msg->size_1);
  // angle_1
  rosidl_runtime_c__double__Sequence__fini(&msg->angle_1);
  // response_1
  rosidl_runtime_c__double__Sequence__fini(&msg->response_1);
  // octave_1
  rosidl_runtime_c__int32__Sequence__fini(&msg->octave_1);
  // class_id_1
  rosidl_runtime_c__int32__Sequence__fini(&msg->class_id_1);
  // x_2
  rosidl_runtime_c__double__Sequence__fini(&msg->x_2);
  // y_2
  rosidl_runtime_c__double__Sequence__fini(&msg->y_2);
  // size_2
  rosidl_runtime_c__double__Sequence__fini(&msg->size_2);
  // angle_2
  rosidl_runtime_c__double__Sequence__fini(&msg->angle_2);
  // response_2
  rosidl_runtime_c__double__Sequence__fini(&msg->response_2);
  // octave_2
  rosidl_runtime_c__int32__Sequence__fini(&msg->octave_2);
  // class_id_2
  rosidl_runtime_c__int32__Sequence__fini(&msg->class_id_2);
}

bool
turtlebot_control__msg__Keypoints__are_equal(const turtlebot_control__msg__Keypoints * lhs, const turtlebot_control__msg__Keypoints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x_1), &(rhs->x_1)))
  {
    return false;
  }
  // y_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y_1), &(rhs->y_1)))
  {
    return false;
  }
  // size_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->size_1), &(rhs->size_1)))
  {
    return false;
  }
  // angle_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->angle_1), &(rhs->angle_1)))
  {
    return false;
  }
  // response_1
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->response_1), &(rhs->response_1)))
  {
    return false;
  }
  // octave_1
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->octave_1), &(rhs->octave_1)))
  {
    return false;
  }
  // class_id_1
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->class_id_1), &(rhs->class_id_1)))
  {
    return false;
  }
  // x_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x_2), &(rhs->x_2)))
  {
    return false;
  }
  // y_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y_2), &(rhs->y_2)))
  {
    return false;
  }
  // size_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->size_2), &(rhs->size_2)))
  {
    return false;
  }
  // angle_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->angle_2), &(rhs->angle_2)))
  {
    return false;
  }
  // response_2
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->response_2), &(rhs->response_2)))
  {
    return false;
  }
  // octave_2
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->octave_2), &(rhs->octave_2)))
  {
    return false;
  }
  // class_id_2
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->class_id_2), &(rhs->class_id_2)))
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
  // x_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x_1), &(output->x_1)))
  {
    return false;
  }
  // y_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y_1), &(output->y_1)))
  {
    return false;
  }
  // size_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->size_1), &(output->size_1)))
  {
    return false;
  }
  // angle_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->angle_1), &(output->angle_1)))
  {
    return false;
  }
  // response_1
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->response_1), &(output->response_1)))
  {
    return false;
  }
  // octave_1
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->octave_1), &(output->octave_1)))
  {
    return false;
  }
  // class_id_1
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->class_id_1), &(output->class_id_1)))
  {
    return false;
  }
  // x_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x_2), &(output->x_2)))
  {
    return false;
  }
  // y_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y_2), &(output->y_2)))
  {
    return false;
  }
  // size_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->size_2), &(output->size_2)))
  {
    return false;
  }
  // angle_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->angle_2), &(output->angle_2)))
  {
    return false;
  }
  // response_2
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->response_2), &(output->response_2)))
  {
    return false;
  }
  // octave_2
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->octave_2), &(output->octave_2)))
  {
    return false;
  }
  // class_id_2
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->class_id_2), &(output->class_id_2)))
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
