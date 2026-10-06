cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
option(PLL_KEEP_DEBUG_SYMBOLS "Retain symbols in the optimized PLL experiment firmware" OFF)
option(PLL_ENABLE_RTT "Enable nonblocking 10 Hz PLL RTT diagnostics" OFF)
option(PLL_AUTO_RUN_ENCODER "Auto-run the 0.10 A encoder baseline after alignment" OFF)

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
if(PLL_KEEP_DEBUG_SYMBOLS)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -O2 -g3)
endif()
if(PLL_ENABLE_RTT)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE PLL_ENABLE_RTT RTT_USE_ASM=0)
endif()
if(PLL_AUTO_RUN_ENCODER)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE PLL_AUTO_RUN_ENCODER)
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Collect user sources recursively
file(GLOB_RECURSE USER_LIB_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/user_lib/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/user_lib/*.cpp
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_LIB_SOURCES}
)

# Keep the 20 kHz estimator optimized even in Debug; retain debug symbols.
set_source_files_properties(
    user_lib/drivers/foc/sensorless/bemf_pll.c
    user_lib/drivers/foc/sensorless/pll_experiment.c
    PROPERTIES COMPILE_OPTIONS "-O2"
)
# The legacy FOC core also runs in this same 50 us ISR. -O0 exceeded the
# deadline on the F407 and silently halved the effective update rate.
file(GLOB PLL_BACKEND_C_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/user_lib/third-parts/SguanFOC/*.c)
set_source_files_properties(${PLL_BACKEND_C_SOURCES}
    user_lib/third-parts/SguanFOC/wrapper/sguan_foc_wrapper.cpp
    user_lib/drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.cpp
    user_lib/system/sys_time.cpp
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    PROPERTIES COMPILE_OPTIONS "-O2")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/user_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/user_lib/third-parts
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate programmer-friendly images together with the ELF file.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating HEX/BIN images and printing firmware size"
    VERBATIM
)

# DAPLink is a CMSIS-DAP probe. Keep flashing optional so hosts without
# OpenOCD can still configure and compile the firmware.
set(OPENOCD_INTERFACE "interface/cmsis-dap.cfg" CACHE STRING
    "OpenOCD interface configuration")
set(OPENOCD_TARGET "target/stm32f4x.cfg" CACHE STRING
    "OpenOCD target configuration")
set(OPENOCD_ADAPTER_SPEED "1000" CACHE STRING
    "SWD clock in kHz")
find_program(OPENOCD_EXECUTABLE NAMES openocd)

if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE}
                -f ${OPENOCD_INTERFACE}
                -c "transport select swd"
                -f ${OPENOCD_TARGET}
                -c "adapter speed ${OPENOCD_ADAPTER_SPEED}"
                -c "program \"$<TARGET_FILE:${CMAKE_PROJECT_NAME}>\" verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing ${CMAKE_PROJECT_NAME}.elf through DAPLink/CMSIS-DAP"
        USES_TERMINAL
        VERBATIM
    )
else()
    message(STATUS "OpenOCD not found: the flash target will not be generated")
endif()
