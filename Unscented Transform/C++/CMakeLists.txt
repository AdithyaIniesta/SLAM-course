cmake_minimum_required(VERSION 3.1)
project(unscented_kalman_filter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include/)

add_library(ukf src/ukf.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main ukf)
