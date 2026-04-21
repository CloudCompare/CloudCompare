find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable(TestArgumentParser
    TestArgumentParser.cpp
)

target_link_libraries(TestArgumentParser
    PRIVATE
        CCPluginAPI
        Qt6::Test
)

add_test(NAME TestArgumentParser COMMAND TestArgumentParser)
