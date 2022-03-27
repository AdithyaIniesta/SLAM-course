cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(extended_kalman_filter)

file(GLOB_RECURSE EKF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(ekf ${CMAKE_CURRENT_SOURCE_DIR} main.cpp ${EKF_SOURCES})

target_include_directories(ekf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
