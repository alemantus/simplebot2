set(OUTPUT_NAME serial_test)
add_executable(${OUTPUT_NAME} serial_test.cpp)

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        )

# enable usb output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

pico_add_extra_outputs(${OUTPUT_NAME})
