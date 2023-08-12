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
# include_directories("/usr/local/include/eigen3/")
include_directories("/usr/include/eigen3/")

add_library(extract_data SHARED extract_data.h extract_data.cpp)
add_library(control SHARED control.h control.cpp)
add_library(actuator SHARED actuator.h actuator.cpp)
add_library(homing SHARED homing.h homing.cpp)
# add_library(utilities SHARED utilities.h)
add_library(GPIO SHARED GPIO/GPIO.h GPIO/GPIO.cpp)
# add_library(wiringPi SHARED /usr/include/wiringPi.h /usr/include/wiringPi.cpp)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

# target_link_libraries(utilities GPIO)
target_link_libraries(homing actuator GPIO)

add_executable(main main.cpp)
add_executable(loggingforhome LoggingForHome.cpp)
target_link_libraries(loggingforhome homing ${WIRINGPI_LIBRARIES})
target_link_libraries(main homing extract_data control ${WIRINGPI_LIBRARIES})

# Create the main executable

