cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(www C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PICO_NO_PICOTOOL 1)

pico_sdk_init()

add_executable(hexapod
        src/main.cpp
        src/tof_manager.cpp
        src/kinematics.cpp
)

target_link_libraries(hexapod
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_gpio
        hardware_timer
        hardware_i2c
        hardware_pio
        pico_multicore
)

target_include_directories(hexapod PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

pico_enable_stdio_usb(hexapod 1)
pico_enable_stdio_uart(hexapod 0)

pico_add_extra_outputs(hexapod)

# Automatische UF2-Erzeugung aus hexapod.bin
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(TARGET hexapod POST_BUILD
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/elf2uf2.py
        $<TARGET_FILE_DIR:hexapod>/hexapod.bin
        $<TARGET_FILE_DIR:hexapod>/hexapod.uf2
        COMMENT "Generiere valide hexapod.uf2 fuer RP2040..."
)