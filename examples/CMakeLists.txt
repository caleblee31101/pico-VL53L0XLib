set(OUTPUT_NAME ranging_examples)
add_executable(${OUTPUT_NAME} main.c)

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    vl53l0x
)
