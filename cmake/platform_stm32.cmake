message(STATUS "Configuring Trifecta for STM32")

# Detect STM32 family if not provided
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

if(NOT DEFINED STM32_SERIES)
    message(FATAL_ERROR "Could not auto-detect STM32 series. Please set STM32_SERIES manually.")
endif()

# CPU flags
if(STM32_SERIES STREQUAL "F4" OR
   STM32_SERIES STREQUAL "L4" OR
   STM32_SERIES STREQUAL "G4")

    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )

elseif(STM32_SERIES STREQUAL "F7" OR
       STM32_SERIES STREQUAL "H7")

    set(CPU_COMPILE_OPTIONS
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
    )

else()
    message(FATAL_ERROR "Unsupported STM32 series: ${STM32_SERIES}")
endif()

# Add STM32-specific sources
file(GLOB_RECURSE TRIFECTA_STM32_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32/${STM32_SERIES}/*.c"
)

target_sources(trifecta_core PRIVATE ${TRIFECTA_STM32_SOURCES})

# Include dirs (HAL/CMSIS must be provided by parent project)
target_include_directories(trifecta_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# Compile flags
target_compile_options(trifecta_core PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -Wall
    -Wextra
    -Wpedantic
    -ffunction-sections
    -fdata-sections
    -Os
)

# Linker flags
target_link_options(trifecta_core PRIVATE
    ${CPU_COMPILE_OPTIONS}
    -Wl,--gc-sections
)

message(STATUS "Building Trifecta for STM32${STM32_SERIES}")
message(STATUS "CPU Options: ${CPU_COMPILE_OPTIONS}")
message(STATUS "STM32 Sources: ${TRIFECTA_STM32_SOURCES}")
