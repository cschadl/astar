cmake_minimum_required(VERSION 3.16)

project(astar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(ASTAR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(GTest REQUIRED)
enable_testing()
include(GoogleTest)

find_package(Threads REQUIRED)

add_subdirectory(examples)
add_subdirectory(tests)