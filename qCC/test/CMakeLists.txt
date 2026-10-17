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

add_executable(TestIndividualFilesCollector
    TestIndividualFilesCollector.cpp
    ../ccIndividualFilesCollector.cpp
)

target_include_directories(TestIndividualFilesCollector
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(TestIndividualFilesCollector
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

add_test(NAME TestIndividualFilesCollector COMMAND TestIndividualFilesCollector)
