# dmrosen 18-May-2017

cmake_minimum_required(VERSION 3.1)

# PROJECT CONFIGURATION
project(SESync LANGUAGES C CXX VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # We require C++ 14

add_compile_options(-Wno-dev)

link_directories(/usr/local/lib)

# Set build type to 'RelWithDebInfo' if one was not specified by the user
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS RelWithDebInfo Release Debug MinSizeRel)
  message(STATUS "Setting build type to ${CMAKE_BUILD_TYPE}, as none was specified\n")
else()
  message(STATUS "Building in ${CMAKE_BUILD_TYPE} mode\n")
endif()

# Directory for built libraries
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib CACHE PATH "The directory in which to place the SE-Sync library built by this project")
# Directory for built executables
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin CACHE PATH "The directory in which to place executables built by this project")

# BUILD CONFIGURATIONS
# Enable faster instruction sets (SIMD/AVX)
set(ENABLE_FAST_INSTRUCTIONS ON CACHE BOOL "Enable faster instruction sets (SIMD/AVX)")
# Enable OpenMP (if available)
set(ENABLE_OPENMP ON CACHE BOOL "Enable OpenMP (if available)")
# Enable code profiling using gperftools
set(ENABLE_PROFILING OFF CACHE BOOL "Enable code profiling using gperftools")

# Add the .cmake files that ship with Eigen3 to the CMake module path (useful for finding other stuff)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/cmake" CACHE STRING "The CMake module path used for this project")

if(${ENABLE_FAST_INSTRUCTIONS})
message(STATUS "Enabling SIMD/AVX instruction sets")
add_definitions(-march=native)
endif()

if(${ENABLE_PROFILING})
message(STATUS "Enabling code profiling using Google Performance Tools")
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -lprofiler)
endif()

message(STATUS "")


# SESYNC DEPENDENCIES

# FIND EIGEN3
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}\n")


# FIND ADDITIONAL LIBRARIES
# These next operations make use of the .cmake files shipped with Eigen3
find_package(SPQR REQUIRED)
find_package(Cholmod REQUIRED)
find_package(BLAS REQUIRED)


# Find Optimization library
set(OPTIMIZATION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Optimization/ CACHE PATH "Path to top-level directory of Optimization (the one containing CMakeLists.txt)")

# Run the Optimization package's CMake file
add_subdirectory(${OPTIMIZATION_DIR})

# PERFORMANCE IMPROVEMENTS
if(${ENABLE_OPENMP})
find_package(OpenMP)
if(OPENMP_FOUND)
message(STATUS "\nFound OpenMP! Turning on support for parallelization\n")
endif()
endif()


# FIND SPECTRA
# Spectra doesn't ship with a .cmake file, but it's header-only, so it suffices to simply determine it's top-level directory
set(SPECTRA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Spectra/include CACHE PATH "Path to top-level directory of Spectra template library")
message(STATUS "Spectra include directory: ${SPECTRA_INCLUDE_DIR}\n")

set(SESync_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SE-Sync/include)
set(SESync_HDR_DIR ${SESync_INCLUDE_DIR}/SESync)
set(SESync_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SE-Sync/src)

set(CPL-SLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CPL-SLAM/include)
set(CPL-SLAM_HDR_DIR ${CPL-SLAM_INCLUDE_DIR}/CPL-SLAM)
set(CPL-SLAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CPL-SLAM/src)

set(EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

# Set the include directories for this project
# Spectra is only used internally by the SE-Sync library
set(SESync_PRIVATE_INCLUDES ${SPECTRA_INCLUDE_DIR} CACHE INTERNAL "")
# The SE-Sync headers and and Eigen 3, SPQR, and Cholmod are all referenced by the header files of the SE-Sync library, hence must be PUBLICLY included (i.e. clients using the SE-Sync headers must also include these headers)
set(SESync_INCLUDES ${SESync_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SPQR_INCLUDES} ${CHOLMOD_INCLUDES} CACHE INTERNAL "")

# Set the include directories for this project
# Spectra is only used internally by the CPL-SLAM library
set(CPL-SLAM_PRIVATE_INCLUDES ${SPECTRA_INCLUDE_DIR} CACHE INTERNAL "")
# The CPL-SLAM headers and and Eigen 3, SPQR, and Cholmod are all referenced by the header files of the CPL-SLAM library, hence must be PUBLICLY included (i.e. clients using the CPL-SLAM headers must also include these headers)
set(CPL-SLAM_INCLUDES ${CPL-SLAM_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${SPQR_INCLUDES} ${CHOLMOD_INCLUDES} CACHE INTERNAL "")


# SE-SYNC PROJECT

# Get the set of SE-Sync header and source files
set(SESync_HDRS
${SESync_HDR_DIR}/StiefelProduct.h
${SESync_HDR_DIR}/RelativePoseMeasurement.h
${SESync_HDR_DIR}/SESync_types.h
${SESync_HDR_DIR}/SESync_utils.h
${SESync_HDR_DIR}/SESyncProblem.h
${SESync_HDR_DIR}/SESync.h
)

set(SESync_SRCS
${SESync_SOURCE_DIR}/StiefelProduct.cpp
${SESync_SOURCE_DIR}/SESync_utils.cpp
${SESync_SOURCE_DIR}/SESyncProblem.cpp
${SESync_SOURCE_DIR}/SESync.cpp
)

# CPL-SYNC PROJECT

# Get the set of CPL-SLAM header and source files
set(CPL-SLAM_HDRS
${CPL-SLAM_HDR_DIR}/Oblique.h
${CPL-SLAM_HDR_DIR}/RelativePoseMeasurement.h
${CPL-SLAM_HDR_DIR}/RelativeLandmarkMeasurement.h
${CPL-SLAM_HDR_DIR}/CPL-SLAM_types.h
${CPL-SLAM_HDR_DIR}/CPL-SLAM_utils.h
${CPL-SLAM_HDR_DIR}/CPL-SLAMProblem.h
${CPL-SLAM_HDR_DIR}/CPL-SLAM.h
)

set(CPL-SLAM_SRCS
${CPL-SLAM_SOURCE_DIR}/Oblique.cpp
${CPL-SLAM_SOURCE_DIR}/CPL-SLAM_utils.cpp
${CPL-SLAM_SOURCE_DIR}/CPL-SLAMProblem.cpp
${CPL-SLAM_SOURCE_DIR}/CPL-SLAM.cpp
)

# Build the SE-Sync library
add_library(SESync SHARED ${SESync_HDRS} ${SESync_SRCS})
target_include_directories(SESync PRIVATE ${SESync_PRIVATE_INCLUDES})
target_include_directories(SESync PUBLIC ${SESync_INCLUDES})
target_link_libraries(SESync Optimization ${BLAS_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES} ${M} ${LAPACK})
install(TARGETS SESync DESTINATION /usr/local/lib)
install(FILES ${SESync_HDRS} DESTINATION /usr/local/include/SESync)

# Build the CPL-SLAM library
add_library(CPL-SLAM SHARED ${CPL-SLAM_HDRS} ${CPL-SLAM_SRCS})
target_include_directories(CPL-SLAM PRIVATE ${CPL-SLAM_PRIVATE_INCLUDES})
target_include_directories(CPL-SLAM PUBLIC ${CPL-SLAM_INCLUDES})
target_link_libraries(CPL-SLAM Optimization ${BLAS_LIBRARIES} ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES} ${M} ${LAPACK})
install(TARGETS CPL-SLAM DESTINATION /usr/local/lib)
install(FILES ${CPL-SLAM_HDRS} DESTINATION /usr/local/include/CPL-SLAM)

if(OPENMP_FOUND)
# Add additional compilation flags to enable OpenMP support
set_target_properties(SESync PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
target_link_libraries(SESync omp5)
set_target_properties(CPL-SLAM PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
target_link_libraries(CPL-SLAM omp5)
endif()

# BUILD EXAMPLE DRIVER
add_subdirectory(examples)
# BUILD ERGODIC LOOP DETECTION TEST
add_subdirectory(eld)
# BUILD THE ROS PACKAGE
# add_subdirectory(spider_ros)



# EXPORT SE-SYNC LIBRARY

# Add add entry for this project into CMake's package registry, so that this project can be found by other CMake projects
export(PACKAGE SESync)
# Create a configuration file for this project, so that it can be imported by other CMake projects
export(TARGETS SESync Optimization FILE SESyncConfig.cmake)

# EXPORT CPL-SYNC LIBRARY

# Add add entry for this project into CMake's package registry, so that this project can be found by other CMake projects
export(PACKAGE CPL-SLAM)
# Create a configuration file for this project, so that it can be imported by other CMake projects
#export(TARGETS CPL-SLAM SESync Optimization FILE CPL-SLAMConfig.cmake)
export(TARGETS CPL-SLAM SESync Optimization FILE CPL-SLAMConfig.cmake)

