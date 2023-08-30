cmake_minimum_required(VERSION 3.15)
project(feature_match)

set(CMAKE_CXX_STANDARD 17)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${PYTHON3_INCLUDE_DIRS}
                    ${NumPy_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/headers)


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

# add_executable(featureMatch src/feature_matching.cpp)
add_executable(image_stitch src/image_stitching.cpp)

# target_include_directories(featureMatch PRIVATE ${PYTHON_INCLUDE_DIRS})

# add_executable(camera_calibration src/camera_calibration.cpp)
# add_executable(capture_image src/capture_image.cpp)
# target_link_libraries(featureMatch ${OpenCV_LIBS} Eigen3::Eigen
# ${PYTHON_LIBRARIES}
# Python3::NumPy)
# target_link_libraries(featureMatch ${OpenCV_LIBS} Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io 
# ${PYTHON_LIBRARIES}
# Python3::NumPy)
# target_link_libraries(camera_calibration ${OpenCV_LIBS} -lopencv_core)
# target_link_libraries(capture_image ${OpenCV_LIBS} -lopencv_core)
target_link_libraries(image_stitch ${OpenCV_LIBS})
# target_link_libraries(teaser_cpp_ply Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io)