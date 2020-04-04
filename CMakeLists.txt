cmake_minimum_required(VERSION 2.8)

project(robot-manipulator)
include_directories(include)
add_library(robot-manipulator src/manipulator.cpp)

add_executable(run-robot-manipulator src/main.cpp)
add_executable(run-tests test/tests.cpp)
target_link_libraries(run-robot-manipulator robot-manipulator)
target_link_libraries(run-tests robot-manipulator)
