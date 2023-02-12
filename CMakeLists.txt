cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_vl53l0x C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/api/core/inc
  ${CMAKE_CURRENT_LIST_DIR}/api/platform/inc
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

set(OUTPUT_NAME ranging_example)

add_executable(${OUTPUT_NAME} examples/main.c)

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    hardware_i2c
)

#target_include_directories(${OUTPUT_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/api/core/inc>
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/api/platform/inc>
#)

target_include_directories(${OUTPUT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/api/core/inc
  ${CMAKE_CURRENT_LIST_DIR}/api/platform/inc
)

target_sources(${OUTPUT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/api/core/src/vl53l0x_api.c
  ${CMAKE_CURRENT_LIST_DIR}/api/core/src/vl53l0x_api_calibration.c
  ${CMAKE_CURRENT_LIST_DIR}/api/core/src/vl53l0x_api_core.c
  ${CMAKE_CURRENT_LIST_DIR}/api/core/src/vl53l0x_api_ranging.c
  ${CMAKE_CURRENT_LIST_DIR}/api/core/src/vl53l0x_api_strings.c
  ${CMAKE_CURRENT_LIST_DIR}/api/platform/src/vl53l0x_platform.c
  # ${CMAKE_CURRENT_LIST_DIR}/api/platform/src/vl53l0x_platform_log.c

)
