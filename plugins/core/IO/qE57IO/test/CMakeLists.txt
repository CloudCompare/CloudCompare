find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable( TestE57Filter )

# the plugin is a shared library that exports nothing: the test builds the sources it needs
target_sources( TestE57Filter
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/TestE57Filter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/E57Filter.cpp
)

target_include_directories( TestE57Filter
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries( TestE57Filter
    PRIVATE
        QCC_DB_LIB
        QCC_IO_LIB
        E57Format
        Qt6::Test
)

if ( USING_STATIC_XERCES )
    target_compile_definitions( TestE57Filter PRIVATE XERCES_STATIC_LIBRARY )
endif()

add_test( NAME TestE57Filter COMMAND TestE57Filter )
