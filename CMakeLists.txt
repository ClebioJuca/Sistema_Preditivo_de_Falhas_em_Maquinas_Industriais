# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Projeto_Final_EmbarcaTech C CXX ASM)

include(FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(SOURCES
    Projeto_Final_EmbarcaTech.c
    lib/ssd1306/ssd1306_i2c.c 
    lib/microfone/microfone.c
    lib/bmp280/bmp280.c
    lib/mpu6050/mpu6050.c
    lib/sd/diskio.c
    lib/sd/ff.c
    lib/sd/ffunicode.c
    lib/sd/sd_card.c
    lib/sd/sd.c
    lib/mqtt/mqtt.c
    )

# Add executable. Default name is the project name, version 0.1

add_executable(Projeto_Final_EmbarcaTech ${SOURCES})

pico_set_program_name(Projeto_Final_EmbarcaTech "Projeto_Final_EmbarcaTech")
pico_set_program_version(Projeto_Final_EmbarcaTech "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Projeto_Final_EmbarcaTech 0)
pico_enable_stdio_usb(Projeto_Final_EmbarcaTech 1)

# Add the standard library to the build
target_link_libraries(Projeto_Final_EmbarcaTech pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# Add the standard include files to the build
target_include_directories(Projeto_Final_EmbarcaTech PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        /lib
)

# Add any user requested libraries
target_link_libraries(Projeto_Final_EmbarcaTech 
        hardware_i2c
        hardware_timer
        hardware_adc
        hardware_spi
        hardware_pwm
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
        )

pico_add_extra_outputs(Projeto_Final_EmbarcaTech)

