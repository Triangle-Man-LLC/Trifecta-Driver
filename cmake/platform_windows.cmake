message(STATUS "Configuring Trifecta Driver for Windows")

set_target_properties(trifecta_core PROPERTIES
    OUTPUT_NAME "DriverTrifectaWindows"
)

# Add Windows-specific sources
target_sources(trifecta_core PRIVATE
    windows/FS_Trifecta_Interfaces.c
    windows/FS_Trifecta_Interfaces_Serial.c
    windows/FS_Trifecta_Interfaces_Networked.c
)

# Windows-specific definitions
target_compile_definitions(trifecta_core PRIVATE
    PLATFORM_WINDOWS
    FS_DRIVER_EXPORTS
)

# Windows-specific libraries
target_link_libraries(trifecta_core PRIVATE ws2_32)

# Optional: force C++ linker if needed
if(CMAKE_CXX_COMPILER)
    set_target_properties(trifecta_core PROPERTIES LINKER_LANGUAGE CXX)
endif()

# Install rules (Windows)
install(TARGETS trifecta_core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
