cmake_minimum_required(VERSION 3.14)
project(robotpilots_algorithm VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(etl REQUIRED)

set(ALGO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(ALGO_SOURCES
    src/algo_crc.cpp
    src/algo_pid.cpp
)

add_library(robotpilots_algorithm STATIC ${ALGO_SOURCES})

target_include_directories(robotpilots_algorithm
    PUBLIC
        $<BUILD_INTERFACE:${ALGO_INCLUDE_DIR}>
)

target_link_libraries(robotpilots_algorithm PUBLIC etl::etl)
