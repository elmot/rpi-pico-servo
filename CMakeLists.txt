cmake_minimum_required(VERSION 3.23)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rpi-pico-servo C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
set(PICO_BOARD pico)

pico_sdk_init()

add_executable(rpi-pico-servo src/servo.c src/pwm.c src/as560x.c)
pico_set_program_name(rpi-pico-servo "rpi-pico-servo")
pico_set_program_version(rpi-pico-servo "0.1")
pico_add_extra_outputs(rpi-pico-servo)
target_compile_options(rpi-pico-servo PRIVATE -g3 -O0)
target_link_options(rpi-pico-servo PRIVATE -g3 -O0)

#pico_generate_pio_header(rp2040-data-logger ${CMAKE_SOURCE_DIR}/src/ws2812.pio OUTPUT_DIR ${CMAKE_SOURCE_DIR}/src/generated)

pico_enable_stdio_usb(rpi-pico-servo 0)
pico_enable_stdio_uart(rpi-pico-servo 1)

target_link_libraries(rpi-pico-servo
        pico_stdlib
        hardware_pwm
        hardware_i2c
        hardware_pio
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_uart
        )

target_include_directories(rpi-pico-servo PRIVATE ${CMAKE_SOURCE_DIR}/src)
#target_include_directories(rpi_europe_ctrl PRIVATE ${CMAKE_SOURCE_DIR}/generated)
