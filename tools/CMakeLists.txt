cmake_minimum_required(VERSION 3.10)
project(can-sensor-tool VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Release build by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Version info passed to source
add_compile_definitions(
    TOOL_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    TOOL_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    TOOL_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# Main executable
add_executable(can-sensor-tool can-sensor-tool.cpp)

# Install target
install(TARGETS can-sensor-tool DESTINATION bin)
