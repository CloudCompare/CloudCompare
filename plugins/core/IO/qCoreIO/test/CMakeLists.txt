find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable( TestCoreIOFilters )

# the plugin is a shared library that exports nothing: the test builds the sources it needs
target_sources( TestCoreIOFilters
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/TestCoreIOFilters.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/ObjFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/OFFFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/PTXFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/SimpleBinFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/STLFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/VTKFilter.cpp
)

target_include_directories( TestCoreIOFilters
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries( TestCoreIOFilters
    PRIVATE
        QCC_DB_LIB
        QCC_IO_LIB
        Qt6::Test
)

add_test( NAME TestCoreIOFilters COMMAND TestCoreIOFilters )
