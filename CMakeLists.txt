cmake_minimum_required(VERSION 3.20)
project(KMR_dxl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(${PROJECT_NAME} 
            source/KMR_dxl_motor_handler.cpp
            source/KMR_dxl_handler.cpp
            source/KMR_dxl_reader.cpp
            source/KMR_dxl_writer.cpp
            source/KMR_dxl_hal.cpp
            source/KMR_dxl_utils.cpp
            )

# Compiler flags
target_compile_options(${PROJECT_NAME}  PRIVATE -Wall -Wno-sign-compare)

include (GNUInstallDirs)
set(MY_PUBLIC_HEADERS
    "include/KMR_dxl.hpp"
)
set(MY_PRIVATE_HEADERS
    "include/*.hpp"
)

#set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${MY_PUBLIC_HEADERS} PRIVATE_HEADER ${MY_PRIVATE_HEADERS})
#install(TARGETS ${PROJECT_NAME}
#        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
#        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
#)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${MY_PUBLIC_HEADERS})
install(TARGETS ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        #PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# Directories containing header files
target_include_directories(${PROJECT_NAME}  PUBLIC include config)

# Locations of the used libraries
target_link_directories(${PROJECT_NAME}  PUBLIC /usr/local/lib)

# Link the used libraries: dynamixel and yaml-cpp
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(${PROJECT_NAME}  dxl_x64_cpp)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(${PROJECT_NAME}  dxl_sbc_cpp)
else()
    message(FATAL_ERROR "ERROR: Unknown processor architecture! CMake will exit")
endif()


# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()


# Generate example executables
add_executable(ex1_position example/ex1_position.cpp)
target_link_libraries(ex1_position KMR_dxl)
add_executable(ex2_speed example/ex2_speed.cpp)
target_link_libraries(ex2_speed KMR_dxl)
add_executable(ex3_current example/ex3_current.cpp)
target_link_libraries(ex3_current KMR_dxl)
add_executable(ex4_pwm example/ex4_pwm.cpp)
target_link_libraries(ex4_pwm KMR_dxl)
add_executable(ex5_multiturn example/ex5_multiturn.cpp)
target_link_libraries(ex5_multiturn KMR_dxl)
#add_executable(ex6_hybrid example/ex6_hybrid.cpp)
#target_link_libraries(ex6_hybrid KMR_dxl)
add_executable(ex7_custom_handlers example/ex7_custom_handlers.cpp)
target_link_libraries(ex7_custom_handlers KMR_dxl)