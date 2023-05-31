// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from img_transform:srv/Teleport.idl
// generated code does not contain a copyright notice
#include "img_transform/srv/detail/teleport__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
img_transform__srv__Teleport_Request__init(img_transform__srv__Teleport_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // theta
  return true;
}

void
img_transform__srv__Teleport_Request__fini(img_transform__srv__Teleport_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // theta
}

bool
img_transform__srv__Teleport_Request__are_equal(const img_transform__srv__Teleport_Request * lhs, const img_transform__srv__Teleport_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  return true;
}

bool
img_transform__srv__Teleport_Request__copy(
  const img_transform__srv__Teleport_Request * input,
  img_transform__srv__Teleport_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  return true;
}

img_transform__srv__Teleport_Request *
img_transform__srv__Teleport_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Request * msg = (img_transform__srv__Teleport_Request *)allocator.allocate(sizeof(img_transform__srv__Teleport_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(img_transform__srv__Teleport_Request));
  bool success = img_transform__srv__Teleport_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
img_transform__srv__Teleport_Request__destroy(img_transform__srv__Teleport_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    img_transform__srv__Teleport_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
img_transform__srv__Teleport_Request__Sequence__init(img_transform__srv__Teleport_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Request * data = NULL;

  if (size) {
    data = (img_transform__srv__Teleport_Request *)allocator.zero_allocate(size, sizeof(img_transform__srv__Teleport_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = img_transform__srv__Teleport_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        img_transform__srv__Teleport_Request__fini(&data[i - 1]);
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
img_transform__srv__Teleport_Request__Sequence__fini(img_transform__srv__Teleport_Request__Sequence * array)
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
      img_transform__srv__Teleport_Request__fini(&array->data[i]);
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

img_transform__srv__Teleport_Request__Sequence *
img_transform__srv__Teleport_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Request__Sequence * array = (img_transform__srv__Teleport_Request__Sequence *)allocator.allocate(sizeof(img_transform__srv__Teleport_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = img_transform__srv__Teleport_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
img_transform__srv__Teleport_Request__Sequence__destroy(img_transform__srv__Teleport_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    img_transform__srv__Teleport_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
img_transform__srv__Teleport_Request__Sequence__are_equal(const img_transform__srv__Teleport_Request__Sequence * lhs, const img_transform__srv__Teleport_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!img_transform__srv__Teleport_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
img_transform__srv__Teleport_Request__Sequence__copy(
  const img_transform__srv__Teleport_Request__Sequence * input,
  img_transform__srv__Teleport_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(img_transform__srv__Teleport_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    img_transform__srv__Teleport_Request * data =
      (img_transform__srv__Teleport_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!img_transform__srv__Teleport_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          img_transform__srv__Teleport_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!img_transform__srv__Teleport_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
img_transform__srv__Teleport_Response__init(img_transform__srv__Teleport_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
img_transform__srv__Teleport_Response__fini(img_transform__srv__Teleport_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
img_transform__srv__Teleport_Response__are_equal(const img_transform__srv__Teleport_Response * lhs, const img_transform__srv__Teleport_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
img_transform__srv__Teleport_Response__copy(
  const img_transform__srv__Teleport_Response * input,
  img_transform__srv__Teleport_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

img_transform__srv__Teleport_Response *
img_transform__srv__Teleport_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Response * msg = (img_transform__srv__Teleport_Response *)allocator.allocate(sizeof(img_transform__srv__Teleport_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(img_transform__srv__Teleport_Response));
  bool success = img_transform__srv__Teleport_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
img_transform__srv__Teleport_Response__destroy(img_transform__srv__Teleport_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    img_transform__srv__Teleport_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
img_transform__srv__Teleport_Response__Sequence__init(img_transform__srv__Teleport_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Response * data = NULL;

  if (size) {
    data = (img_transform__srv__Teleport_Response *)allocator.zero_allocate(size, sizeof(img_transform__srv__Teleport_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = img_transform__srv__Teleport_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        img_transform__srv__Teleport_Response__fini(&data[i - 1]);
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
img_transform__srv__Teleport_Response__Sequence__fini(img_transform__srv__Teleport_Response__Sequence * array)
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
      img_transform__srv__Teleport_Response__fini(&array->data[i]);
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

img_transform__srv__Teleport_Response__Sequence *
img_transform__srv__Teleport_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  img_transform__srv__Teleport_Response__Sequence * array = (img_transform__srv__Teleport_Response__Sequence *)allocator.allocate(sizeof(img_transform__srv__Teleport_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = img_transform__srv__Teleport_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
img_transform__srv__Teleport_Response__Sequence__destroy(img_transform__srv__Teleport_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    img_transform__srv__Teleport_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
img_transform__srv__Teleport_Response__Sequence__are_equal(const img_transform__srv__Teleport_Response__Sequence * lhs, const img_transform__srv__Teleport_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!img_transform__srv__Teleport_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
img_transform__srv__Teleport_Response__Sequence__copy(
  const img_transform__srv__Teleport_Response__Sequence * input,
  img_transform__srv__Teleport_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(img_transform__srv__Teleport_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    img_transform__srv__Teleport_Response * data =
      (img_transform__srv__Teleport_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!img_transform__srv__Teleport_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          img_transform__srv__Teleport_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!img_transform__srv__Teleport_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
