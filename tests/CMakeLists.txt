cmake_minimum_required(VERSION 3.10)
project(TrifectaDriverTests C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

# Bring in the actual Trifecta-Driver module
add_subdirectory(${CMAKE_SOURCE_DIR}/../ TrifectaDriver_build)

# Collect all the CSV files that are used for testing
file(GLOB TEST_CSVS "${CMAKE_SOURCE_DIR}/test_resources/*.csv")
foreach(csv ${TEST_CSVS})
    get_filename_component(name "${csv}" NAME)
    configure_file("${csv}" "${CMAKE_BINARY_DIR}/test_resources/${name}" COPYONLY)
endforeach()

# Collect all test .c files
file(GLOB TEST_SOURCES *.c)
message(STATUS "Test sources: ${TEST_SOURCES}")
foreach(TEST_FILE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_FILE})

    target_link_libraries(${TEST_NAME}
        PRIVATE
            DriverTrifecta
            m
    )

    target_compile_options(${TEST_NAME} PRIVATE -g3 -O0 -fsanitize=address,undefined)
    target_link_options(${TEST_NAME} PRIVATE -g3 -fsanitize=address,undefined)

    add_test(
        NAME ${TEST_NAME}
        COMMAND /usr/bin/time -v $<TARGET_FILE:${TEST_NAME}>
    )
endforeach()
