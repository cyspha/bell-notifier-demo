#cmake_minimum_required(VERSION 3.21)
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(bell_notifier_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(bell_notifier main.c)

pico_enable_stdio_usb(bell_notifier 1)
pico_enable_stdio_uart(bell_notifier 1)

pico_add_extra_outputs(bell_notifier)

target_link_libraries(bell_notifier pico_stdlib)
target_link_libraries(bell_notifier hardware_adc)
target_link_libraries(bell_notifier hardware_timer)

pico_enable_stdio_usb(bell_notifier 1)
pico_enable_stdio_uart(bell_notifier 0)