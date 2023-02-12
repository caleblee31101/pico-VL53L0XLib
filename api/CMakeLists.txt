set(LIRBARY_NAME vl53l0x)
add_library(${LIRBARY_NAME} INTERFACE)

target_include_directories(${LIRBARY_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/core/inc
  ${CMAKE_CURRENT_LIST_DIR}/platform/inc
  )

target_sources(${LIRBARY_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api.c
  ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_calibration.c
  ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_core.c
  ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_ranging.c
  ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_strings.c
  ${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l0x_platform.c
)

# Pull in pico libraries that we need
target_link_libraries(${LIRBARY_NAME} INTERFACE pico_stdlib hardware_i2c)
