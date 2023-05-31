cmake_minimum_required(VERSION 3.8)
project(img_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msg "msg/Keypoints.msg" LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")

add_executable(img_transform src/img_transform.cpp)

add_executable(video_frequency src/video_frequency.cpp)

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

# include_directories(${OpenCV_INCLUDE_DIRS}
#                     ${PYTHON3_INCLUDE_DIRS}
#                     ${NumPy_INCLUDE_DIRS}
#                     ${CMAKE_SOURCE_DIR}/headers)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(img_transform ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
)

target_link_libraries(video_frequency ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  ${PYTHON_LIBRARIES}
  Python3::NumPy
)

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
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  images
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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