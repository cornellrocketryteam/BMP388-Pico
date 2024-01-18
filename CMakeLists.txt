add_library(BMP388-Pico INTERFACE)

target_sources(BMP388-Pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bmp388.cpp
)

target_link_libraries(BMP388-Pico INTERFACE
    hardware_i2c
    pico_stdlib
)