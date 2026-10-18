cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(robotpilots_algorithm VERSION 1.0.0 LANGUAGES CXX)
endif()

option(ALGO_ENABLE_ARM_MATH "Build algorithms that depend on CMSIS-DSP" OFF)

if(NOT ALGO_ZEPHYR AND NOT TARGET etl::etl)
    find_package(etl CONFIG REQUIRED)
endif()

set(ALGO_ENABLE_ARM_MATH_EFFECTIVE OFF)
if(ALGO_ENABLE_ARM_MATH OR CONFIG_ALGO_ENABLE_ARM_MATH)
    set(ALGO_ENABLE_ARM_MATH_EFFECTIVE ON)
endif()

set(ALGO_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

set(ALGO_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/algo_crc.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/algo_pid.cpp
)

if(ALGO_ENABLE_ARM_MATH_EFFECTIVE)
    list(APPEND ALGO_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/src/algo_ekf_filter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/algo_kf_filter.cpp
    )
endif()

if(ALGO_ZEPHYR)
    zephyr_library_named(robotpilots_algorithm)
    zephyr_library_sources(${ALGO_SOURCES})
    target_include_directories(robotpilots_algorithm
        PUBLIC
            ${ALGO_INCLUDE_DIR}
    )
    zephyr_include_directories(${ZEPHYR_ETL_MODULE_DIR}/include)

    if(ALGO_ENABLE_ARM_MATH_EFFECTIVE)
        zephyr_library_compile_definitions(ALGO_ENABLE_ARM_MATH)
    endif()
else()
    add_library(robotpilots_algorithm STATIC ${ALGO_SOURCES})

    if(ALGO_ENABLE_ARM_MATH_EFFECTIVE)
        target_compile_definitions(robotpilots_algorithm PUBLIC ALGO_ENABLE_ARM_MATH)
    endif()

    target_link_libraries(robotpilots_algorithm PUBLIC etl::etl)

    target_include_directories(robotpilots_algorithm
        PUBLIC
            $<BUILD_INTERFACE:${ALGO_INCLUDE_DIR}>
    )
endif()

add_library(robotpilots::algorithm ALIAS robotpilots_algorithm)
