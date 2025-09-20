cmake_minimum_required(VERSION 3.10)

# Project name
project(DriverTrifectaWindows C)

# Set the version number
set(DriverTrifectaWindows_VERSION_MAJOR 1)
set(DriverTrifectaWindows_VERSION_MINOR 0)

# Specify the C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Add the library
add_library(DriverTrifectaWindows
    common/FS_Trifecta.c
    common/FS_Trifecta_Device.c
    common/FS_Trifecta_Device_Utils.c
    common/FS_Trifecta_Serial.c
    common/FS_Trifecta_Networked.c
    windows/FS_Trifecta_Interfaces.c
)

# Include directories
target_include_directories(DriverTrifectaWindows PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optional: Windows-specific definitions
target_compile_definitions(DriverTrifectaWindows PRIVATE PLATFORM_WINDOWS)

# Optional: Set linker language to C++ if needed
if(CMAKE_CXX_COMPILER)
    set_target_properties(DriverTrifectaWindows PROPERTIES
        LINKER_LANGUAGE CXX
    )
endif()

# Ensure the build/cmake directory exists
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/cmake")

# Generate DriverTrifectaWindowsConfig.cmake.in
file(WRITE "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaWindowsConfig.cmake.in" "
@PACKAGE_INIT@

include(\"\${CMAKE_CURRENT_LIST_DIR}/DriverTrifectaWindowsTargets.cmake\")
")

# Create and install the configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfigVersion.cmake"
    VERSION ${DriverTrifectaWindows_VERSION_MAJOR}.${DriverTrifectaWindows_VERSION_MINOR}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaWindowsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfig.cmake"
    INSTALL_DESTINATION lib/cmake/DriverTrifectaWindows
)

# Install the library and headers
install(TARGETS DriverTrifectaWindows
        EXPORT DriverTrifectaWindowsTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Install export targets
install(EXPORT DriverTrifectaWindowsTargets
        FILE DriverTrifectaWindowsTargets.cmake
        DESTINATION lib/cmake/DriverTrifectaWindows)

# Install configuration files
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfigVersion.cmake"
        DESTINATION lib/cmake/DriverTrifectaWindows)

# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
