set(OUTPUT_NAME simplebot)
add_executable(${OUTPUT_NAME} simplebot.cpp)

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        motor2040
        button
        pid
        )

# enable usb output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

pico_add_extra_outputs(${OUTPUT_NAME})
