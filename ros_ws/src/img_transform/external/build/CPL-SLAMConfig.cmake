# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.6)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.20)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

if(CMAKE_VERSION VERSION_LESS 3.0.0)
  message(FATAL_ERROR "This file relies on consumers using CMake 3.0.0 or greater.")
endif()

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget CPL-SLAM SESync Optimization)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target CPL-SLAM
add_library(CPL-SLAM SHARED IMPORTED)

set_target_properties(CPL-SLAM PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Eigen"
  INTERFACE_LINK_LIBRARIES "Optimization;/usr/lib/x86_64-linux-gnu/libopenblas.so;/usr/local/lib/libcholmod.so.4.0.4;/usr/local/lib/libspqr.so.3.0.4;omp5"
)

# Create imported target SESync
add_library(SESync SHARED IMPORTED)

set_target_properties(SESync PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Eigen"
  INTERFACE_LINK_LIBRARIES "Optimization;/usr/lib/x86_64-linux-gnu/libopenblas.so;/usr/local/lib/libcholmod.so.4.0.4;/usr/local/lib/libspqr.so.3.0.4;omp5"
)

# Create imported target Optimization
add_library(Optimization INTERFACE IMPORTED)

set_target_properties(Optimization PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include"
  INTERFACE_SOURCES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Base/Concepts.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Util/Stopwatch.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Smooth/Concepts.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Smooth/GradientDescent.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Smooth/TNT.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Convex/Concepts.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Convex/ProximalGradient.h;/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/Optimization/include/Optimization/Convex/ADMM.h"
)

# Import target "CPL-SLAM" for configuration "RelWithDebInfo"
set_property(TARGET CPL-SLAM APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(CPL-SLAM PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/lib/libCPL-SLAM.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libCPL-SLAM.so"
  )

# Import target "SESync" for configuration "RelWithDebInfo"
set_property(TARGET SESync APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(SESync PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/lib/libSESync.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libSESync.so"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)