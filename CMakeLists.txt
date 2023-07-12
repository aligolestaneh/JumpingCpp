cmake_minimum_required(VERSION 3.10)
project(Jump)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(YAML_LIBRARY /usr/lib/x86_64-linux-gnu)

# find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# include_directories("/usr/local/include/lcm/")
# include_directories("/usr/local/include/eigen3/")
include_directories("/usr/include/yaml-cpp")


# add_library(extract_data SHARED extract_data.h extract_data.cpp)
# add_library(test SHARED test.cpp)

add_executable(test test.cpp)

target_link_libraries(test ${YAML_LIBRARY})

# Create the main executable

# target_link_libraries(hopping_for_new_robot PUBLIC extract_data)
