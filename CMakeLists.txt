cmake_minimum_required(VERSION 3.20)
project(KMR_dxl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(KMR_dxl 
            source/KMR_dxl_robot.cpp
            source/KMR_dxl_handler.cpp
            source/KMR_dxl_reader.cpp
            source/KMR_dxl_writer.cpp
            source/KMR_dxl_hal.cpp
            source/KMR_dxl_utils.cpp
            )

# Directories containing header files
target_include_directories(KMR_dxl PUBLIC include config)

# Locations of the used libraries
target_link_directories(KMR_dxl PUBLIC /usr/local/lib)

# Link the used libraries: dynamixel and yaml-cpp
target_link_libraries(KMR_dxl yaml-cpp)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(KMR_dxl dxl_x64_cpp)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(KMR_dxl dxl_sbc_cpp)
else()
    message(FATAL_ERROR "ERROR: Unknown processor architecture! CMake will exit")
endif()


# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()
