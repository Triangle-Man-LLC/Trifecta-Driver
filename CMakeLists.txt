cmake_minimum_required(VERSION 3.10)

# Project name
project(DriverTrifectaAndroid C)

# Set the version number
set(DriverTrifectaAndroid_VERSION_MAJOR 1)
set(DriverTrifectaAndroid_VERSION_MINOR 0)

# Specify the C standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

# Option to build shared library
option(SHAREDLIB "Build DriverTrifectaAndroid as a shared library" OFF)

# Set library type based on option
set(LIB_TYPE STATIC)
if(SHAREDLIB)
    set(LIB_TYPE SHARED)
endif()

# Add the library
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

# Include directories
target_include_directories(DriverTrifectaAndroid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the math library
target_link_libraries(DriverTrifectaAndroid m)

# # For C++ compatibility, set extern "C"
# if(CMAKE_CXX_COMPILER)
#     set_target_properties(DriverTrifectaAndroid PROPERTIES
#         LINKER_LANGUAGE CXX
#     )
# endif()

# Ensure the build/cmake directory exists
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/cmake")

# Generate DriverTrifectaAndroidConfig.cmake.in
file(WRITE "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaAndroidConfig.cmake.in" "
@PACKAGE_INIT@

include(\"\${CMAKE_CURRENT_LIST_DIR}/DriverTrifectaAndroidTargets.cmake\")
")

# Create and install the configuration files
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

# Install the library and headers
install(TARGETS DriverTrifectaAndroid
        EXPORT DriverTrifectaAndroidTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Install export targets
install(EXPORT DriverTrifectaAndroidTargets
        FILE DriverTrifectaAndroidTargets.cmake
        DESTINATION lib/cmake/DriverTrifectaAndroid)

# Install configuration files
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaAndroidConfigVersion.cmake"
        DESTINATION lib/cmake/DriverTrifectaAndroid)

# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)

#
# --- Packaging for Android driver distribution ---
#

# Output root
set(DRIVER_OUT_DIR "${CMAKE_BINARY_DIR}/Trifecta-Driver-Android")

# Output subfolders
set(DRIVER_OUT_LIB_DIR "${DRIVER_OUT_DIR}/lib")
set(DRIVER_OUT_SRC_DIR "${DRIVER_OUT_DIR}/src")

# Ensure directories exist
file(MAKE_DIRECTORY "${DRIVER_OUT_LIB_DIR}")
file(MAKE_DIRECTORY "${DRIVER_OUT_SRC_DIR}")

# Copy the .so after build
add_custom_command(TARGET DriverTrifectaAndroid POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:DriverTrifectaAndroid>"
        "${DRIVER_OUT_LIB_DIR}/libTrifectaDriverAndroid.so"
    COMMENT "Copying libTrifectaDriverAndroid.so to Trifecta-Driver-Android/lib/"
)

# Copy Java API files
file(GLOB TRIFECTA_JAVA_FILES
    "${CMAKE_SOURCE_DIR}/java/*.java"
)

add_custom_target(copy_java ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRIFECTA_JAVA_FILES}
        "${DRIVER_OUT_SRC_DIR}"
    COMMENT "Copying Java API files to Trifecta-Driver-Android/src/"
)

add_dependencies(copy_java DriverTrifectaAndroid)
