// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot_control:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__FUNCTIONS_H_
#define TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot_control/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot_control/msg/detail/keypoints__struct.h"

/// Initialize msg/Keypoints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot_control__msg__Keypoints
 * )) before or use
 * turtlebot_control__msg__Keypoints__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__init(turtlebot_control__msg__Keypoints * msg);

/// Finalize msg/Keypoints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
void
turtlebot_control__msg__Keypoints__fini(turtlebot_control__msg__Keypoints * msg);

/// Create msg/Keypoints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot_control__msg__Keypoints__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
turtlebot_control__msg__Keypoints *
turtlebot_control__msg__Keypoints__create();

/// Destroy msg/Keypoints message.
/**
 * It calls
 * turtlebot_control__msg__Keypoints__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
void
turtlebot_control__msg__Keypoints__destroy(turtlebot_control__msg__Keypoints * msg);

/// Check for msg/Keypoints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__are_equal(const turtlebot_control__msg__Keypoints * lhs, const turtlebot_control__msg__Keypoints * rhs);

/// Copy a msg/Keypoints message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__copy(
  const turtlebot_control__msg__Keypoints * input,
  turtlebot_control__msg__Keypoints * output);

/// Initialize array of msg/Keypoints messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot_control__msg__Keypoints__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__Sequence__init(turtlebot_control__msg__Keypoints__Sequence * array, size_t size);

/// Finalize array of msg/Keypoints messages.
/**
 * It calls
 * turtlebot_control__msg__Keypoints__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
void
turtlebot_control__msg__Keypoints__Sequence__fini(turtlebot_control__msg__Keypoints__Sequence * array);

/// Create array of msg/Keypoints messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot_control__msg__Keypoints__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
turtlebot_control__msg__Keypoints__Sequence *
turtlebot_control__msg__Keypoints__Sequence__create(size_t size);

/// Destroy array of msg/Keypoints messages.
/**
 * It calls
 * turtlebot_control__msg__Keypoints__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
void
turtlebot_control__msg__Keypoints__Sequence__destroy(turtlebot_control__msg__Keypoints__Sequence * array);

/// Check for msg/Keypoints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__Sequence__are_equal(const turtlebot_control__msg__Keypoints__Sequence * lhs, const turtlebot_control__msg__Keypoints__Sequence * rhs);

/// Copy an array of msg/Keypoints messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot_control
bool
turtlebot_control__msg__Keypoints__Sequence__copy(
  const turtlebot_control__msg__Keypoints__Sequence * input,
  turtlebot_control__msg__Keypoints__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT_CONTROL__MSG__DETAIL__KEYPOINTS__FUNCTIONS_H_
