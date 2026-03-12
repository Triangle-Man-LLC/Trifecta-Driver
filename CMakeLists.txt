cmake_minimum_required(VERSION 3.20)
project(DriverTrifecta LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

if (CMAKE_SYSTEM_NAME STREQUAL "ESP-IDF")
    include(cmake/platform_espidf.cmake)
    return()
endif()

set(TRIFECTA_COMMON_SRCS
    common/FS_Trifecta.c
    common/FS_Trifecta_Device.c
    common/FS_Trifecta_Device_Utils.c
    common/FS_Trifecta_Serial.c
    common/FS_Trifecta_Networked.c
)

add_library(trifecta_core ${TRIFECTA_COMMON_SRCS})

target_include_directories(trifecta_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Platform dispatcher
if (WIN32)
    include(cmake/platform_windows.cmake)

elseif (UNIX AND NOT APPLE AND NOT ANDROID)
    include(cmake/platform_linux.cmake)

elseif (ANDROID)
    include(cmake/platform_android.cmake)

elseif (CMAKE_SYSTEM_NAME STREQUAL "Generic")
    include(cmake/platform_stm32.cmake)

else()
    message(FATAL_ERROR "Unsupported platform")
endif()
