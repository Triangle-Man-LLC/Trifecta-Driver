message(STATUS "Configuring Trifecta Driver for Linux")

set_target_properties(trifecta_core PROPERTIES
    OUTPUT_NAME "DriverTrifecta"
)

# Add Linux-specific sources
target_sources(trifecta_core PRIVATE
    linux/FS_Trifecta_Interfaces.c
    linux/FS_Trifecta_Interfaces_Serial.c
    linux/FS_Trifecta_Interfaces_Networked.c
)

# Linux-specific definitions
target_compile_definitions(trifecta_core PRIVATE PLATFORM_LINUX)

# Linux-specific libraries
target_link_libraries(trifecta_core PRIVATE m)

# Optional: force C++ linker if needed
if(CMAKE_CXX_COMPILER)
    set_target_properties(trifecta_core PROPERTIES LINKER_LANGUAGE CXX)
endif()

# Install rules (Linux)
install(TARGETS trifecta_core
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
