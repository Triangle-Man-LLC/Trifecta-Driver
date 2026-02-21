cmake_minimum_required(VERSION 3.10)

project(DriverTrifectaWindows C)

set(DriverTrifectaWindows_VERSION_MAJOR 1)
set(DriverTrifectaWindows_VERSION_MINOR 0)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

# Build as a shared library (.dll)
add_library(DriverTrifectaWindows SHARED
    common/FS_Trifecta.c
    common/FS_Trifecta_Device.c
    common/FS_Trifecta_Device_Utils.c
    common/FS_Trifecta_Serial.c
    common/FS_Trifecta_Networked.c
    windows/FS_Trifecta_Interfaces.c
    windows/FS_Trifecta_Interfaces_Serial.c
    windows/FS_Trifecta_Interfaces_Networked.c
)

# Include directories
target_include_directories(DriverTrifectaWindows PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Windows-specific definitions
target_compile_definitions(DriverTrifectaWindows PRIVATE
    PLATFORM_WINDOWS
    FS_DRIVER_EXPORTS  # Enables __declspec(dllexport)
)

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

# Install the DLL and headers
install(TARGETS DriverTrifectaWindows
        EXPORT DriverTrifectaWindowsTargets
        RUNTIME DESTINATION bin       # .dll
        LIBRARY DESTINATION lib       # .lib (import)
        ARCHIVE DESTINATION lib       # .lib (static, if built)
        INCLUDES DESTINATION include)

install(EXPORT DriverTrifectaWindowsTargets
        FILE DriverTrifectaWindowsTargets.cmake
        DESTINATION lib/cmake/DriverTrifectaWindows)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/DriverTrifectaWindowsConfigVersion.cmake"
        DESTINATION lib/cmake/DriverTrifectaWindows)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
