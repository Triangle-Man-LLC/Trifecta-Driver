message(STATUS "Configuring Trifecta Driver for Linux")

set_target_properties(DriverTrifecta PROPERTIES
    OUTPUT_NAME "DriverTrifecta"
)

# Add Linux-specific sources
target_sources(DriverTrifecta PRIVATE
    linux/FS_Trifecta_Interfaces.c
    linux/FS_Trifecta_Interfaces_Serial.c
    linux/FS_Trifecta_Interfaces_Networked.c
)

# Linux-specific definitions
target_compile_definitions(DriverTrifecta PRIVATE PLATFORM_LINUX)

# Linux-specific libraries
target_link_libraries(DriverTrifecta PRIVATE m)

# Optional: force C++ linker if needed
if(CMAKE_CXX_COMPILER)
    set_target_properties(DriverTrifecta PROPERTIES LINKER_LANGUAGE CXX)
endif()

# Install rules (Linux)
install(TARGETS DriverTrifecta
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
