cmake_minimum_required(VERSION 3.13)
project(Trifecta-Driver LANGUAGES C)

# User-configurable STM32 series (default: F4)
set(STM32_SERIES "F4" CACHE STRING "STM32 series (F1, F4, H7, etc.)")
set_property(CACHE STM32_SERIES PROPERTY STRINGS F1 F4 H7)

# Find required packages
find_package(CMSIS REQUIRED)
find_package(HAL REQUIRED)

# Configure compiler options based on series
if(STM32_SERIES STREQUAL "F1")
    set(CPU_COMPILE_OPTIONS 
        -mcpu=cortex-m3
        -mthumb
    )
    set(LINK_LIBRARIES
        CMSIS::STM32::F1
        HAL::STM32::F1::GPIO
        HAL::STM32::F1::SPI
    )
elseif(STM32_SERIES STREQUAL "F4")
    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )
    set(LINK_LIBRARIES
        CMSIS::STM32::F4
        HAL::STM32::F4::GPIO
        HAL::STM32::F4::SPI
    )
elseif(STM32_SERIES STREQUAL "H7")
    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
    )
    set(LINK_LIBRARIES
        CMSIS::STM32::H7
        HAL::STM32::H7::GPIO
        HAL::STM32::H7::SPI
    )
else()
    message(FATAL_ERROR "Unsupported STM32 series: ${STM32_SERIES}")
endif()

# Define source files
file(GLOB_RECURSE SOURCES
    "common/*.c"
    "stm32/*.c"
)

# Define the component library
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC ${LINK_LIBRARIES})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -Wall
    -Wextra
    -Wpedantic
    -ffunction-sections
    -fdata-sections
    -Os
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -specs=nosys.specs
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
)

# Print configuration summary
message(STATUS "Building for STM32${STM32_SERIES} series")
message(STATUS "CPU Options: ${CPU_COMPILE_OPTIONS}")
message(STATUS "Linked Libraries: ${LINK_LIBRARIES}")
