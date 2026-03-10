pico_sdk_init()

add_executable(rtd_node
    rtd_node.cpp
)

// Link Pico SDK libraries
target_link_libraries(rtd_node
    pico_stdlib
    hardware_gpio
    hardware_timer
    //TODO: add your libcan library here, e.g.:
    //libcan
)

//Enable USB serial output for debug (disable in production if desired)
pico_enable_stdio_usb(rtd_node 1)
pico_enable_stdio_uart(rtd_node 0)

//Generate .uf2 and other flashable outputs
pico_add_extra_outputs(rtd_node)