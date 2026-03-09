find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable( TestLasExtraBytesEvlr )

# the plugin is a shared library that exports nothing: the test builds the sources it needs
target_sources( TestLasExtraBytesEvlr
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/TestLasExtraBytesEvlr.cpp
        ${CMAKE_CURRENT_LIST_DIR}/TestLasExtraBytesEvlr.h
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasDetails.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasExtraScalarField.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasScalarField.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasScalarFieldSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasWaveformSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasMetadata.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/LasVlr.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../src/CopcLoader.cpp
)

target_include_directories( TestLasExtraBytesEvlr
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries( TestLasExtraBytesEvlr
    PRIVATE
        QCC_DB_LIB
        QCC_IO_LIB
        LASzip::LASzip
        Qt6::Test
)

if ( WIN32 )
    set_target_properties( TestLasExtraBytesEvlr PROPERTIES
        WIN32_EXECUTABLE False
    )
endif()

add_test( NAME TestLasExtraBytesEvlr COMMAND TestLasExtraBytesEvlr )
