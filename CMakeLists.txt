cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(robotpilots_algorithm VERSION 1.0.0 LANGUAGES CXX)
endif()

option(ALGO_ENABLE_ARM_MATH "Build algorithms that depend on CMSIS-DSP" OFF)

if(NOT ALGO_ZEPHYR AND NOT TARGET etl::etl)
    find_package(etl CONFIG REQUIRED)
endif()

if(ALGO_ENABLE_ARM_MATH OR CONFIG_ALGO_ENABLE_ARM_MATH)
	add_compile_definitions(ALGO_ENABLE_ARM_MATH)
endif()

set(ALGO_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

file(GLOB ALGO_SOURCES
	"${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
)

if(ALGO_ZEPHYR)
    zephyr_library_named(robotpilots_algorithm)
    zephyr_library_sources(${ALGO_SOURCES})
    target_include_directories(robotpilots_algorithm
        PUBLIC
            ${ALGO_INCLUDE_DIR}
    )
    zephyr_include_directories(${ZEPHYR_ETL_MODULE_DIR}/include)

else()
    add_library(robotpilots_algorithm STATIC ${ALGO_SOURCES})

    target_link_libraries(robotpilots_algorithm PUBLIC etl::etl)

    target_include_directories(robotpilots_algorithm
        PUBLIC
            $<BUILD_INTERFACE:${ALGO_INCLUDE_DIR}>
    )
endif()

add_library(robotpilots::algorithm ALIAS robotpilots_algorithm)
