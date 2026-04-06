cmake_minimum_required(VERSION 3.20)

# Detect ESP-IDF build
if(DEFINED COMPONENT_DIR)
    message(STATUS "ESP-IDF build detected")
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/platform_espidf.cmake)
    return()

else()

    project(DriverTrifecta LANGUAGES C CXX)

    set(CMAKE_C_STANDARD 17)
    set(CMAKE_C_STANDARD_REQUIRED ON)

    set(TRIFECTA_COMMON_SRCS
        common/FS_Trifecta.c
        common/FS_Trifecta_Device.c
        common/FS_Trifecta_Device_Utils.c
        common/FS_Trifecta_Networked.c
        common/FS_Trifecta_Serial.c
        common/FS_Trifecta_Replay.c
    )

    if(BUILD_KIND STREQUAL "static")
        set(BUILD_LIBRARY_TYPE STATIC)
    elseif(BUILD_KIND STREQUAL "shared")
        set(BUILD_LIBRARY_TYPE SHARED)
    else()
        set(BUILD_LIBRARY_TYPE STATIC)
    endif()

    add_library(DriverTrifecta ${BUILD_LIBRARY_TYPE} ${TRIFECTA_COMMON_SRCS})

    target_include_directories(DriverTrifecta PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    # Platform dispatcher
    if (WIN32)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/platform_windows.cmake)

    elseif (ANDROID_NDK)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/platform_android.cmake)

    elseif (UNIX AND NOT APPLE AND NOT ANDROID)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/platform_linux.cmake)

    elseif (CMAKE_SYSTEM_NAME STREQUAL "Generic")
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/platform_stm32.cmake)

    else()
        message(FATAL_ERROR "Unsupported platform")
    endif()

endif()
