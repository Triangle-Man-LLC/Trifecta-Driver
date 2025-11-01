cmake_minimum_required(VERSION 3.10)

# Project name
project(DriverTrifectaLinux C)

# Set the version number
set(DriverTrifectaLinux_VERSION_MAJOR 1)
set(DriverTrifectaLinux_VERSION_MINOR 0)

# Specify the C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Option to build shared library
option(SHAREDLIB "Build DriverTrifectaLinux as a shared library" OFF)

# Set library type based on option
set(LIB_TYPE STATIC)
if(SHAREDLIB)
    set(LIB_TYPE SHARED)
endif()

# Add the library
add_library(DriverTrifectaLinux ${LIB_TYPE}
    common/FS_Trifecta.c
    common/FS_Trifecta_Device.c
    common/FS_Trifecta_Device_Utils.c
    common/FS_Trifecta_Serial.c
    common/FS_Trifecta_Networked.c
    linux/FS_Trifecta_Interfaces.c
    linux/FS_Trifecta_Interfaces_Serial.c
    linux/FS_Trifecta_Interfaces_Networked.c
)

# Include directories
target_include_directories(DriverTrifectaLinux PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the math library
target_link_libraries(DriverTrifectaLinux m)

# For C++ compatibility, set extern "C"
if(CMAKE_CXX_COMPILER)
    set_target_properties(DriverTrifectaLinux PROPERTIES
        LINKER_LANGUAGE CXX
    )
endif()

# Ensure the build/cmake directory exists
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/cmake")

# Generate DriverTrifectaLinuxConfig.cmake.in
file(WRITE "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaLinuxConfig.cmake.in" "
@PACKAGE_INIT@

include(\"\${CMAKE_CURRENT_LIST_DIR}/DriverTrifectaLinuxTargets.cmake\")
")

# Create and install the configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaLinuxConfigVersion.cmake"
    VERSION ${DriverTrifectaLinux_VERSION_MAJOR}.${DriverTrifectaLinux_VERSION_MINOR}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_BINARY_DIR}/cmake/DriverTrifectaLinuxConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaLinuxConfig.cmake"
    INSTALL_DESTINATION lib/cmake/DriverTrifectaLinux
)

# Install the library and headers
install(TARGETS DriverTrifectaLinux
        EXPORT DriverTrifectaLinuxTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Install export targets
install(EXPORT DriverTrifectaLinuxTargets
        FILE DriverTrifectaLinuxTargets.cmake
        DESTINATION lib/cmake/DriverTrifectaLinux)

# Install configuration files
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaLinuxConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaLinuxConfigVersion.cmake"
        DESTINATION lib/cmake/DriverTrifectaLinux)

# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
