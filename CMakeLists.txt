cmake_minimum_required(VERSION 3.10)

#  Detect superbuild mode
#  - If ANDROID_ABI is NOT set -> superbuild
#  - If ANDROID_ABI IS set -> normal driver build

if(NOT ANDROID_ABI)

    project(DriverTrifectaAndroidSuperbuild C)

    if(NOT ANDROID_NDK)
        message(FATAL_ERROR "You must pass -DANDROID_NDK=/path/to/ndk")
    endif()

    message(STATUS "Superbuild mode enabled")
    message(STATUS "Using Android NDK: ${ANDROID_NDK}")

    # Output root
    set(DRIVER_OUT_DIR "${CMAKE_BINARY_DIR}/Trifecta-Driver-Android")
    set(DRIVER_OUT_SRC_DIR "${DRIVER_OUT_DIR}/src")

    file(MAKE_DIRECTORY "${DRIVER_OUT_DIR}")
    file(MAKE_DIRECTORY "${DRIVER_OUT_SRC_DIR}")

    # Copy Java API files once
    file(GLOB TRIFECTA_JAVA_FILES
        "${CMAKE_SOURCE_DIR}/java/*.java"
    )

    add_custom_target(copy_java ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${TRIFECTA_JAVA_FILES}
            "${DRIVER_OUT_SRC_DIR}"
        COMMENT "Copying Java API files to Trifecta-Driver-Android/src/"
    )

    # ABIs to build
    set(ANDROID_ABIS arm64-v8a armeabi-v7a x86_64)

    add_custom_target(build_all_abies ALL)

    foreach(ABI ${ANDROID_ABIS})

        set(BUILD_DIR "${CMAKE_BINARY_DIR}/build-${ABI}")
        set(ABI_OUT_DIR "${DRIVER_OUT_DIR}/lib/${ABI}")

        file(MAKE_DIRECTORY "${ABI_OUT_DIR}")

        add_custom_command(
            OUTPUT "${BUILD_DIR}/stamp"
            COMMAND ${CMAKE_COMMAND}
                -S ${CMAKE_SOURCE_DIR}
                -B ${BUILD_DIR}
                -DCMAKE_TOOLCHAIN_FILE=${ANDROID_NDK}/build/cmake/android.toolchain.cmake
                -DANDROID_ABI=${ABI}
                -DANDROID_PLATFORM=21
                -DSHAREDLIB=ON
                -DANDROID_NDK=${ANDROID_NDK}
            COMMAND ${CMAKE_COMMAND} --build ${BUILD_DIR} --config Release
            COMMAND ${CMAKE_COMMAND} -E copy
                "${BUILD_DIR}/libDriverTrifectaAndroid.so"
                "${ABI_OUT_DIR}/libTrifectaDriverAndroid.so"
            COMMAND ${CMAKE_COMMAND} -E touch "${BUILD_DIR}/stamp"
            COMMENT "Building ABI ${ABI}"
        )

        add_custom_target(build_${ABI} DEPENDS "${BUILD_DIR}/stamp")
        add_dependencies(build_all_abies build_${ABI})

    endforeach()

    return()
endif()

#
#  Normal single‑ABI driver build (used by superbuild)
#

project(DriverTrifectaAndroid C)

set(DriverTrifectaAndroid_VERSION_MAJOR 1)
set(DriverTrifectaAndroid_VERSION_MINOR 0)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

option(SHAREDLIB "Build DriverTrifectaAndroid as a shared library" OFF)

set(LIB_TYPE STATIC)
if(SHAREDLIB)
    set(LIB_TYPE SHARED)
endif()

add_library(DriverTrifectaAndroid ${LIB_TYPE}
    common/FS_Trifecta.c
    common/FS_Trifecta_Device.c
    common/FS_Trifecta_Device_Utils.c
    common/FS_Trifecta_Serial.c
    common/FS_Trifecta_Networked.c
    android/FS_Trifecta_Interfaces.c
    android/FS_Trifecta_Interfaces_Serial.c
    android/FS_Trifecta_Interfaces_Networked.c
    android/FS_Trifecta_JNI_Packet_Unpack.c
)

target_include_directories(DriverTrifectaAndroid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(DriverTrifectaAndroid m)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/cmake")

file(WRITE "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaAndroidConfig.cmake.in" "
@PACKAGE_INIT@
include(\"\${CMAKE_CURRENT_LIST_DIR}/DriverTrifectaAndroidTargets.cmake\")
")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfigVersion.cmake"
    VERSION ${DriverTrifectaAndroid_VERSION_MAJOR}.${DriverTrifectaAndroid_VERSION_MINOR}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaAndroidConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfig.cmake"
    INSTALL_DESTINATION lib/cmake/DriverTrifectaAndroid
)

install(TARGETS DriverTrifectaAndroid
        EXPORT DriverTrifectaAndroidTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(EXPORT DriverTrifectaAndroidTargets
        FILE DriverTrifectaAndroidTargets.cmake
        DESTINATION lib/cmake/DriverTrifectaAndroid)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfigVersion.cmake"
        DESTINATION lib/cmake/DriverTrifectaAndroid)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)

#
# Packaging for Android driver distribution (single ABI)
#

set(DRIVER_OUT_DIR "${CMAKE_BINARY_DIR}/Trifecta-Driver-Android")

if(ANDROID_ABI)
    set(DRIVER_OUT_LIB_DIR "${DRIVER_OUT_DIR}/lib/${ANDROID_ABI}")
else()
    set(DRIVER_OUT_LIB_DIR "${DRIVER_OUT_DIR}/lib")
endif()

file(MAKE_DIRECTORY "${DRIVER_OUT_LIB_DIR}")

set(DRIVER_OUT_SRC_DIR "${DRIVER_OUT_DIR}/src")
file(MAKE_DIRECTORY "${DRIVER_OUT_SRC_DIR}")

add_custom_command(TARGET DriverTrifectaAndroid POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:DriverTrifectaAndroid>"
        "${DRIVER_OUT_LIB_DIR}/libTrifectaDriverAndroid.so"
    COMMENT "Copying libTrifectaDriverAndroid.so to Trifecta-Driver-Android/lib/${ANDROID_ABI}/"
)

file(GLOB TRIFECTA_JAVA_FILES
    "${CMAKE_SOURCE_DIR}/java/*.java"
)

add_custom_target(copy_java_single ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRIFECTA_JAVA_FILES}
        "${DRIVER_OUT_SRC_DIR}"
    COMMENT "Copying Java API files to Trifecta-Driver-Android/src/"
)

add_dependencies(copy_java_single DriverTrifectaAndroid)
