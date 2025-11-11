cmake_minimum_required(VERSION 3.13)
project(Trifecta-Driver LANGUAGES C)

# If the top-level project already defined the STM32 variant, use it
if(DEFINED STM32_FAMILY)
    set(STM32_SERIES ${STM32_FAMILY})
endif()

# Else, auto-detect from HAL/CMSIS include paths
if(NOT DEFINED STM32_SERIES)
    foreach(dir ${CMAKE_INCLUDE_PATH})
        if(dir MATCHES "STM32F4")
            set(STM32_SERIES "F4")
        elseif(dir MATCHES "STM32H7")
            set(STM32_SERIES "H7")
        elseif(dir MATCHES "STM32L4")
            set(STM32_SERIES "L4")
        elseif(dir MATCHES "STM32G4")
            set(STM32_SERIES "G4")
        elseif(dir MATCHES "STM32F7")
            set(STM32_SERIES "F7")
        endif()
    endforeach()
endif()

# Fallback error in case the STM32 version was not detected
if(NOT DEFINED STM32_SERIES)
    message(FATAL_ERROR "Could not auto-detect STM32 series. Please set STM32_SERIES manually.")
endif()

if(STM32_SERIES STREQUAL "F4" OR
   STM32_SERIES STREQUAL "L4" OR
   STM32_SERIES STREQUAL "G4")

    # Cortex-M4F
    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )

elseif(STM32_SERIES STREQUAL "F7" OR
       STM32_SERIES STREQUAL "H7")

    # Cortex-M7F
    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
    )

else()
    message(FATAL_ERROR "Unsupported STM32 series: ${STM32_SERIES}")
endif()


file(GLOB_RECURSE TRIFECTA_COMMON_SOURCES
    "common/*.c"
)

file(GLOB_RECURSE TRIFECTA_STM32_SOURCES
    "stm32/${STM32_SERIES}/*.c"
)

add_library(${PROJECT_NAME} STATIC
    ${TRIFECTA_COMMON_SOURCES}
    ${TRIFECTA_STM32_SOURCES}
)

# Note: The top-level project needs to provide HAL and CMSIS
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -Wall
    -Wextra
    -Wpedantic
    -ffunction-sections
    -fdata-sections
    -Os
)

target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -Wl,--gc-sections
)

# Print out some debug information
message(STATUS "Building Trifecta-Driver for STM32${STM32_SERIES}")
message(STATUS "CPU Options: ${CPU_COMPILE_OPTIONS}")
message(STATUS "Common Sources: ${TRIFECTA_COMMON_SOURCES}")
message(STATUS "STM32 Sources: ${TRIFECTA_STM32_SOURCES}")
