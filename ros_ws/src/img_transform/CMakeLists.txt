cmake_minimum_required(VERSION 3.8)
project(img_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH /path/to/spqr/installation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(Sophus REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(SPQR CONFIG REQUIRED PATHS "/home/megsindelar/Final_Project/SuiteSparse/SPQR/cmake_modules")
# find_package(CHOLMOD CONFIG REQUIRED PATHS "/home/megsindelar/Final_Project/SuiteSparse/CHOLMOD/cmake_modules")
# find_package(SuiteSparse REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



# set(SPQR_LIBRARIES "/path/to/SPQR/lib/libspqr.so")  # Adjust the library path and name as necessary




# # Locate CPL_SLAM library
# find_path(SPINNAKER_INCLUDE_DIR NAMES Spinnaker.h PATHS /opt/spinnaker/include)
# find_library(SPINNAKER_LIBRARY NAMES Spinnaker PATHS /opt/spinnaker/lib)

# # Check if Spinnaker library and header files are found
# if (SPINNAKER_INCLUDE_DIR AND SPINNAKER_LIBRARY)
#     set(SPINNAKER_FOUND TRUE)
# else()
#     set(SPINNAKER_FOUND FALSE)
# endif()

# # Provide information about Spinnaker to CMake
# if (SPINNAKER_FOUND)
#     if (NOT Spinnaker_FOUND)
#         set(Spinnaker_FOUND TRUE)
#         set(Spinnaker_INCLUDE_DIRS ${SPINNAKER_INCLUDE_DIR})
#         set(Spinnaker_LIBRARIES ${SPINNAKER_LIBRARY})
#     endif()
# endif()
# message("SPINNAKER_FOUND: ${SPINNAKER_FOUND}")

# # Provide variables to the user
# mark_as_advanced(SPINNAKER_INCLUDE_DIR SPINNAKER_LIBRARY)

# include(external/CMakeLists.txt)

add_subdirectory(external)



rosidl_generate_interfaces(${PROJECT_NAME}_msg "msg/Keypoints.msg" "msg/Transform.msg" LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")

include_directories(include external/Optimization/include external/SE-Sync/include)

add_executable(img_transform src/img_transform.cpp)

add_executable(video_frequency src/video_frequency.cpp)

add_executable(wheel_control src/wheel_control.cpp)

add_executable(cpl_slam_node src/cpl_slam_node.cpp)

add_library(transform src/transform.cpp)

ament_target_dependencies(img_transform
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Eigen3
  teaserpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  OpenCV
  Sophus
  message_filters
  nav_msgs
)

ament_target_dependencies(video_frequency
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Eigen3
  teaserpp
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(wheel_control
  rclcpp
  std_msgs
  geometry_msgs
  nuturtlebot_msgs
  std_srvs
)

ament_target_dependencies(cpl_slam_node
  rclcpp
  std_msgs
  geometry_msgs
  nuturtlebot_msgs
  std_srvs
)


include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(img_transform ${OpenCV_LIBS} 
  Eigen3::Eigen
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  Sophus::Sophus
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
)

target_link_libraries(video_frequency ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
)

target_link_libraries(wheel_control ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
)

target_link_libraries(cpl_slam_node ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
)

target_link_libraries(transform Eigen3::Eigen)

target_compile_features(transform PUBLIC cxx_std_17)

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

install(TARGETS
  img_transform
  video_frequency
  wheel_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()