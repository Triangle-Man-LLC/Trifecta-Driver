cmake_minimum_required(VERSION 3.10)
project(TrifectaDriverTests C)

# Version info
set(TrifectaDriverTests_VERSION_MAJOR 1)
set(TrifectaDriverTests_VERSION_MINOR 0)

# C standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)

# Explicitly list required common sources
set(COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/../common/FS_Trifecta_Device_Utils.c
    ${CMAKE_SOURCE_DIR}/../common/FS_Trifecta_Device.c
)

# Include headers from ../include
include_directories(${CMAKE_SOURCE_DIR}/../include ${CMAKE_SOURCE_DIR}/../)

# Enable testing
enable_testing()

# Collect all test .c files in this directory
file(GLOB TEST_SOURCES *.c)
message(STATUS "Test sources: ${TEST_SOURCES}")

# Loop through each test file and create an executable + test
foreach(TEST_FILE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_FILE} ${COMMON_SOURCES})
    target_link_libraries(${TEST_NAME} m)  # Link against libm
    
    # Enable AddressSanitizer
    target_compile_options(${TEST_NAME} PRIVATE -fsanitize=address,undefined)
    target_link_options(${TEST_NAME} PRIVATE -fsanitize=address,undefined)

    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
