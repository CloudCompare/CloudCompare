find_package(Qt6 REQUIRED COMPONENTS Test)

# Command-line argument parser tests
add_executable(TestArgumentParser
    TestArgumentParser.cpp
)

target_link_libraries(TestArgumentParser
    PRIVATE
        CCPluginAPI
        Qt6::Test
)

add_test(NAME TestArgumentParser COMMAND TestArgumentParser)

# Pure projection/exposure tests; also buildable directly without Qt.
add_executable(TestXRayExposure TestXRayExposure.cpp)
target_include_directories(TestXRayExposure PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/qCC_db/include)
add_test(NAME TestXRayExposure COMMAND TestXRayExposure)

# Render real mixed scenes, including the EDL compatibility path.
add_executable(TestXRayDisplay
    TestXRayDisplay.cpp
    ../../plugins/core/GL/qEDL/src/ccEDLFilter.cpp
)
target_include_directories(TestXRayDisplay PRIVATE ../../plugins/core/GL/qEDL/include)
target_link_libraries(TestXRayDisplay PRIVATE QCC_GL_LIB CCAppCommon Qt6::Test)
file(COPY
    ../shaders/ColorRamp
    ../shaders/DrawNormals
    ../../libs/CCFbo/shaders/Bilateral
    ../../plugins/core/GL/qEDL/shaders/EDL
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/shaders
)
target_compile_definitions(TestXRayDisplay PRIVATE
    TEST_SHADER_PATH="${CMAKE_CURRENT_BINARY_DIR}/shaders"
)
add_test(NAME TestXRayDisplay COMMAND TestXRayDisplay
    -o -,txt -o ${CMAKE_CURRENT_BINARY_DIR}/TestXRayDisplay.log,txt)
