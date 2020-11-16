# Main library
add_library(superpixel_mesh)
target_include_directories(superpixel_mesh
    PUBLIC
        $<INSTALL_INTERFACE:.>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/.>
)

# Ceres solver
find_package(Ceres REQUIRED)
target_link_libraries(superpixel_mesh
    PUBLIC
        Ceres::ceres
)

target_sources(superpixel_mesh
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/mesh.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/image.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/meshing_options.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/report.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/meshing_cost.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/superpixel_mesh.hpp>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/superpixel_mesh/superpixel_mesh.cpp
    )

target_compile_features(superpixel_mesh
    PUBLIC
        cxx_std_17
)

add_library(SuperpixelMesh::SuperpixelMesh ALIAS superpixel_mesh)

# include install rules
include(${CMAKE_CURRENT_LIST_DIR}/cmake/SuperpixelMeshInstall.cmake.in)
