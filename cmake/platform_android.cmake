# cmake/platform_android.cmake

#
#  Detect superbuild mode
#  - If ANDROID_ABI is NOT set -> superbuild
#  - If ANDROID_ABI IS set -> normal driver build
#

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
                "${BUILD_DIR}/Trifecta-Driver-Android/lib/${ABI}/libTrifectaDriverAndroid.so"
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

set(DriverTrifectaAndroid_VERSION_MAJOR 1)
set(DriverTrifectaAndroid_VERSION_MINOR 0)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

option(SHAREDLIB "Build DriverTrifectaAndroid as a shared library" OFF)

set(LIB_TYPE SHARED)
# if(SHAREDLIB)
#     set(LIB_TYPE SHARED)
# endif()

message(STATUS "Android single-ABI build")

# Android-specific sources on the core driver target
target_sources(DriverTrifecta PRIVATE
    android/FS_Trifecta_Interfaces.c
    android/FS_Trifecta_Interfaces_Serial.c
    android/FS_Trifecta_Interfaces_Networked.c
    android/FS_Trifecta_JNI_Packet_Unpack.c
)

target_compile_definitions(DriverTrifecta PRIVATE PLATFORM_ANDROID)

# Consistent keyword signature
target_link_libraries(DriverTrifecta PRIVATE m)

# Android includes are implementation detail, not INTERFACE
target_include_directories(DriverTrifecta PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set_target_properties(DriverTrifecta PROPERTIES
    OUTPUT_NAME "TrifectaDriverAndroid"
)

# Put the .so directly in the build dir root so superbuild can find it
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

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

add_custom_command(TARGET DriverTrifecta POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:DriverTrifecta>"
        "${DRIVER_OUT_LIB_DIR}/libTrifectaDriverAndroid.so"
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

add_dependencies(copy_java_single DriverTrifecta)
