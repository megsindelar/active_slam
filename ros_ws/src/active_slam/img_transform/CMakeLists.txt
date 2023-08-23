cmake_minimum_required(VERSION 3.8)
project(img_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH /path/to/spqr/installation)

# find_library(LIB_DBOW2 libDBoW2.so PATHS /usr/local/lib)

# if(NOT LIB_DBOW2)
#     message(FATAL_ERROR "libDBoW2 library not found")
# endif()

# set(LIB_DBOW2_LIBRARIES ${LIB_DBOW2})

SET(CMAKE_EXE_LINKER_FLAGS 
          "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/usr/local/lib")


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
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(DBoW2 REQUIRED)

include_directories(include)


rosidl_generate_interfaces(${PROJECT_NAME}_msg "msg/Odom.msg" "msg/Transform.msg" "msg/Nodes.msg" "msg/FrameID.msg" "msg/Waypoint.msg" LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")

add_library(transform src/transform.cpp)

add_executable(img_transform src/img_transform.cpp)

add_executable(wheel_encoder src/wheel_encoder.cpp)

add_executable(registration src/registration.cpp)

add_executable(se_sync src/se_sync.cpp)

add_executable(active_learning src/active_learning.cpp)

add_subdirectory(external)

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
  nav_msgs
)

ament_target_dependencies(wheel_encoder
  rclcpp
  std_msgs
  geometry_msgs
  nuturtlebot_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  turtlelib
  tf2
  tf2_ros
  Sophus
)

ament_target_dependencies(registration
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
  nav_msgs
  std_srvs
  visualization_msgs
  DBoW2
)

ament_target_dependencies(se_sync
  rclcpp
  std_msgs
  geometry_msgs
  nuturtlebot_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  turtlelib
  tf2
  tf2_ros
  std_srvs
)

ament_target_dependencies(active_learning
  rclcpp
  std_msgs
  geometry_msgs
  nuturtlebot_msgs
  std_srvs
  nav_msgs
  visualization_msgs
  turtlelib
  tf2
  tf2_ros
  std_srvs
)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
  ${SESync_INCLUDE_DIR}
)

link_directories("usr/local/lib")

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


target_link_libraries(wheel_encoder ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  Sophus::Sophus
  "${cpp_typesupport_target}"
  Optimization
  turtlelib::turtlelib
)

target_link_libraries(registration ${OpenCV_LIBS} 
  Eigen3::Eigen
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  Sophus::Sophus
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
  turtlelib::turtlelib
  DBoW2
)

target_link_libraries(se_sync ${OpenCV_LIBS} 
  Eigen3::Eigen 
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  Sophus::Sophus
  transform
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
  Optimization
  SESync
  turtlelib::turtlelib
)

target_link_libraries(active_learning ${OpenCV_LIBS} 
  Eigen3::Eigen 
  Sophus::Sophus
  ${PYTHON_LIBRARIES}
  Python3::NumPy
  "${cpp_typesupport_target}"
  Optimization
  SESync
  turtlelib::turtlelib
)


target_link_libraries(transform
  Eigen3::Eigen
  SESync
)


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
  se_sync
  wheel_encoder
  registration
  active_learning
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  transform
  EXPORT transform-targets
)


install(DIRECTORY
  launch
  config
  data
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