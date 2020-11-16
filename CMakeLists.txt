cmake_minimum_required(VERSION 3.16)
project(superpixel_mesh VERSION 0.1.0 LANGUAGES CXX)

# Options
option(SUPERPIXEL_MESH_BUILD_APPS "Build demo apps" ON)
option(SUPERPIXEL_MESH_BUILD_TESTS "Build tests (requires GoogleTest)" OFF)
set(SUPERPIXEL_MESH_TEST_OUTPUT "." CACHE STRING "Folder to place test generated files")

# Default build to Release
set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build to default type '${default_build_type}'.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build.")
endif ()

add_subdirectory(lib)
if (SUPERPIXEL_MESH_BUILD_APPS)
    add_subdirectory(apps)
endif ()

# Testing
if (SUPERPIXEL_MESH_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif ()
