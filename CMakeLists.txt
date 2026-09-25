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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MotorControl C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(MotorControl MotorControl.c)

# Generate PIO header targeting MotorControl
pico_generate_pio_header(MotorControl ${CMAKE_CURRENT_LIST_DIR}/encoder_period.pio)

pico_set_program_name(MotorControl "MotorControl")
pico_set_program_version(MotorControl "0.1")

# Enable USB output
pico_enable_stdio_uart(MotorControl 0)
pico_enable_stdio_usb(MotorControl 1)

# Add include directories
target_include_directories(MotorControl PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link required libraries
target_link_libraries(MotorControl
    pico_stdlib
    hardware_pio
    hardware_pwm
    hardware_adc
    hardware_timer
    pico_cyw43_arch_none
)

pico_add_extra_outputs(MotorControl)