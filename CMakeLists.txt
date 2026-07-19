cmake_minimum_required(VERSION 3.20)
project(robotpilots_algorithm VERSION 1.0.0 LANGUAGES CXX)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

set(ALGO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(ALGO_SOURCES
    src/algo_crc.cpp
    src/algo_pid.cpp
)

add_library(robotpilots_algorithm STATIC ${ALGO_SOURCES})

target_link_libraries(robotpilots_algorithm PUBLIC zephyr_interface)

target_include_directories(robotpilots_algorithm
    PUBLIC
        $<BUILD_INTERFACE:${ALGO_INCLUDE_DIR}>
)
