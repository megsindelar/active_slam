# Install script for directory: /home/megsindelar/Final_Project/ros_ws/src/img_transform/external

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libSESync.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libSESync.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libSESync.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libSESync.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/lib/libSESync.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libSESync.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libSESync.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/lib/libSESync.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libSESync.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/SESync/StiefelProduct.h;/usr/local/include/SESync/RelativePoseMeasurement.h;/usr/local/include/SESync/SESync_types.h;/usr/local/include/SESync/SESync_utils.h;/usr/local/include/SESync/SESyncProblem.h;/usr/local/include/SESync/SESync.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/SESync" TYPE FILE FILES
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/StiefelProduct.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/RelativePoseMeasurement.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/SESync_types.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/SESync_utils.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/SESyncProblem.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/SE-Sync/include/SESync/SESync.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libCPL-SLAM.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/lib/libCPL-SLAM.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libCPL-SLAM.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/CPL-SLAM/Oblique.h;/usr/local/include/CPL-SLAM/RelativePoseMeasurement.h;/usr/local/include/CPL-SLAM/RelativeLandmarkMeasurement.h;/usr/local/include/CPL-SLAM/CPL-SLAM_types.h;/usr/local/include/CPL-SLAM/CPL-SLAM_utils.h;/usr/local/include/CPL-SLAM/CPL-SLAMProblem.h;/usr/local/include/CPL-SLAM/CPL-SLAM.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/CPL-SLAM" TYPE FILE FILES
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/Oblique.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/RelativePoseMeasurement.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/RelativeLandmarkMeasurement.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/CPL-SLAM_types.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/CPL-SLAM_utils.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/CPL-SLAMProblem.h"
    "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/CPL-SLAM/include/CPL-SLAM/CPL-SLAM.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/Optimization/cmake_install.cmake")
  include("/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/examples/cmake_install.cmake")
  include("/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/eld/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/megsindelar/Final_Project/ros_ws/src/img_transform/external/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
