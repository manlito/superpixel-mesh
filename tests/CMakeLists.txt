include(GoogleTest)

set(LIB_LINK_LIBRARIES
    SuperpixelMesh::SuperpixelMesh
    GTest::GTest
    GTest::Main
)
set(LIB_DEFINITIONS
    TESTS_OUTPUT_DIR="${SUPERPIXEL_MESH_TEST_OUTPUT}"
)

# Synthetic tests
add_executable(test_synthetic test_io.cpp)
target_link_libraries(test_synthetic PRIVATE ${LIB_LINK_LIBRARIES})
target_compile_definitions(test_synthetic PRIVATE ${LIB_DEFINITIONS})
target_compile_features(test_synthetic PRIVATE cxx_std_17)
gtest_discover_tests(test_synthetic)
