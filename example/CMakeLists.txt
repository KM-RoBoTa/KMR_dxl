cmake_minimum_required(VERSION 3.20.2)

# Set the project name
project(example1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)


if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    set(USING_RASPBERRY 0)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    set(USING_RASPBERRY 1)
else()
    message(FATAL_ERROR "ERROR: Unknown processor architecture! CMake will exit")
endif()

# Create a variable to hold source files
set(SOURCE_FILES
    example1.cpp
)

# Add your source files and create the executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Locations of the used libraries
target_link_directories(${PROJECT_NAME} PUBLIC ../build/)

# Directories containing header files
target_include_directories(${PROJECT_NAME}  PUBLIC ../include)

target_link_libraries(${PROJECT_NAME} KMR_dxl)