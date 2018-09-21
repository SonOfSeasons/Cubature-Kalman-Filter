cmake_minimum_required (VERSION 3.0)
add_definitions(-std=c++11)
project (example)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} include)
file(GLOB SOURCES "src/*.cpp")
add_executable(example ${SOURCES})


