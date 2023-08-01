cmake_minimum_required(VERSION 3.9)
project(turtlelib)

find_package(Armadillo REQUIRED)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(frame_main src/frame_main.cpp)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(turtlelib src/rigid2d.cpp src/diff_drive.cpp src/discriminant.cpp src/circle_detection.cpp)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
target_link_libraries(frame_main turtlelib ${ARMADILLO_LIBRARIES})

# specify additional compilation flags for a target
# enable C++ 17
target_compile_features(turtlelib PUBLIC cxx_std_17)


if(NOT CMAKE_CROSSCOMPILING)
# CMake also has the ability to generate doxygen documentation
find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)
endif()

# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)

enable_testing()
add_executable(turtlelib_test tests/tests.cpp)
target_link_libraries(turtlelib_test turtlelib Catch2::Catch2WithMain ${ARMADILLO_LIBRARIES})

add_test(NAME Test_of_Turtlelib COMMAND turtlelib_test)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the 
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# warnings are your friend!
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

# install the include directories
install(DIRECTORY include/turtlelib DESTINATION include)

# Install the targets and create a CMake Exported Target
# The CMake Exported Target can be used to access this project 
# from other CMake projects, after installation
# The targets will be installed to default locations
install(TARGETS frame_main turtlelib EXPORT turtlelib-targets)

# The project_name-targets now also needs to be exported.
# This call will generate a file called project_name-config.cmake
# That contains the exported targets.
# After installation this file will then be found when calling 
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use your library
install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})