# Set minimum required version of Cmake
cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(RACKET C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ===== Biblioteka ICM20948 =====
add_library(icm20948
    src/icm20948.c
)

target_include_directories(icm20948 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(icm20948
    pico_stdlib
    hardware_i2c
)

# ===== Program główny =====
add_executable(${PROJECT_NAME}
    src/main.c
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    icm20948
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)


# Podczas pierwszego uruchamiania trzeba:
# cmake .. -G "Ninja" -DCMAKE_EXPORT_COMPILE_COMMANDS=1
