cmake_minimum_required(VERSION 3.20)
project(DEVIL5-RobotCode LANGUAGES C CXX)

# Modern C++
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include source and header files
include_directories(
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/include
)

file(GLOB_RECURSE SOURCES
        src/*.c
        src/*.cpp
        include/*.h
        include/*.hpp
)

# Dummy build target for Intellisense purposes
if (NOT CMAKE_CROSSCOMPILING)
    add_executable(dummy_index ${SOURCES})
    set_target_properties(dummy_index PROPERTIES
            LINKER_LANGUAGE CXX
            EXCLUDE_FROM_ALL TRUE
            CXX_STANDARD 20
            CXX_STANDARD_REQUIRED YES
            CXX_EXTENSIONS NO
    )
endif ()

# Actual build target that points to PROS's Makefile
add_custom_target(vex_build ALL
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Building project with PROS Makefile"
)