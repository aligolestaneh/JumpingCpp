cmake_minimum_required(VERSION 3.10)
project(Jump)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(YAML_LIBRARY /usr/lib/x86_64-linux-gnu)

find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)

# include_directories("/usr/local/include/lcm/")
# include_directories("/usr/include/yaml-cpp")

include_directories("/usr/include/")
include_directories("/usr/local/include/eigen3/")
# include_directories("/usr/include/eigen3/")		//for raspberry


add_library(extract_data SHARED extract_data.h extract_data.cpp)
add_library(control SHARED control.h control.cpp)
add_library(actuator SHARED actuator.h actuator.cpp)
add_library(homing SHARED homing.h homing.cpp)
# add_library(utilities SHARED utilities.h)
# add_library(GPIO SHARED /usr/include/GPIO.h)

# target_link_libraries(utilities GPIO)
target_link_libraries(homing actuator)

add_executable(main main.cpp)
target_link_libraries(main homing extract_data control)

# Create the main executable

