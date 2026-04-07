find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable(TestArgumentParser
    TestArgumentParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../ccArgumentParser.cpp
)

target_include_directories(TestArgumentParser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(TestArgumentParser
    PRIVATE
        QCC_DB_LIB
        Qt6::Test
)

add_test(NAME TestArgumentParser COMMAND TestArgumentParser)
