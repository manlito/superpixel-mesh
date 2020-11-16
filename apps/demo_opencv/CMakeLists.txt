cmake_minimum_required(VERSION 3.16)
project(demo_opencv CXX)

add_executable(demo_opencv)

find_package(OpenCV COMPONENTS imgcodecs core videoio highgui REQUIRED)
target_include_directories(demo_opencv
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(demo_opencv
    PUBLIC
        ${OpenCV_LIBS}
)

target_link_libraries(demo_opencv
    PUBLIC
        SuperpixelMesh::SuperpixelMesh
)

target_include_directories(demo_opencv
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(demo_opencv
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/draw_mesh.hpp>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/draw_mesh.cpp
)
