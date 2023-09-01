cmake_minimum_required(VERSION 3.20)
project(KMR_dxl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(KMR_dxl 
            source/lib_robot.cpp
            source/lib_dxl_handler.cpp
            source/lib_dxl_reader.cpp
            source/lib_dxl_writer.cpp
            source/lib_hal.cpp)

# Directories containing header files
target_include_directories(KMR_dxl PUBLIC include)

# Locations of the used libraries
target_link_directories(KMR_dxl PRIVATE /usr/local/include/dynamixel_sdk)

# Link the used libraries: dynamixel and yaml-cpp
target_link_libraries(KMR_dxl yaml-cpp dxl_x64_cpp)
