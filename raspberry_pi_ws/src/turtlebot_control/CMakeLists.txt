cmake_minimum_required(VERSION 3.8)
project(turtlebot_control)

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
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msg
  "msg/Keypoints.msg"
  LIBRARY_NAME ${PROJECT_NAME}
 )
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_msg 
  "rosidl_typesupport_cpp"
 )

add_executable(turtlebot_control src/turtlebot_control.cpp)

# include_directories(include)
# add_executable(cam2image src/cam2image.cpp)
# add_executable(cam2img src/cam2img.cpp)

ament_target_dependencies(turtlebot_control
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Eigen3
)


# ament_target_dependencies(cam2image
#   rclcpp
#   std_msgs
#   sensor_msgs
#   cv_bridge
#   image_transport
#   Eigen3
# )

# ament_target_dependencies(cam2img
#   rclcpp
#   std_msgs
#   sensor_msgs
#   cv_bridge
#   image_transport
#   Eigen3
# )

# include_directories(${OpenCV_INCLUDE_DIRS}
#                     ${PYTHON3_INCLUDE_DIRS}
#                     ${NumPy_INCLUDE_DIRS}
#                     ${CMAKE_SOURCE_DIR}/headers)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(turtlebot_control
  Eigen3::Eigen
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

# target_link_libraries(cam2image
#   ${OpenCV_LIBS}
#   "${cpp_typesupport_target}"
# )

# target_link_libraries(cam2img
#   ${OpenCV_LIBS}
#   "${cpp_typesupport_target}"
# )

install(TARGETS
  turtlebot_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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