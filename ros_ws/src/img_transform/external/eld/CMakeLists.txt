cmake_minimum_required(VERSION 3.16)
project(ergodic_loop_sampling)

# compile as c++ 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disable GNU extensions so the program remains compliant
# with C++ standards and can also run on other system
set(CMAKE_CXX_EXTENSIONS OFF)

# enable all warning messages
# add_compile_options(-Wall -Wextra)
add_compile_options(-Wno-dev)

# add executable
add_executable(${PROJECT_NAME} eld_main.cpp)
target_link_libraries(${PROJECT_NAME} SESync CPL-SLAM)

# code profiling with gperftools
if(${ENABLE_PROFILING})
    target_compile_definitions(CPL-SLAM PRIVATE GPREFTOOLS)
    target_link_libraries(CPL-SLAM profiler)
endif()

# openmp support
if(${ENABLE_OPENMP})
    if(OPENMP_FOUND)
        set_target_properties(CPL-SLAM PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
    endif()
endif()

message(STATUS "Building Ergodic-Loop-Detection test executable in directory: ${EXECUTABLE_OUTPUT_PATH}\n")

