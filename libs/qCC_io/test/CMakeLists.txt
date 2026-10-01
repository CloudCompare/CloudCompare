find_package(Qt6 REQUIRED COMPONENTS Test)

add_executable( TestPlyFilter )

target_sources( TestPlyFilter
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/TestPlyFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/TestPlyFilter.h
)

target_link_libraries( TestPlyFilter
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

if ( WIN32 )
    set_target_properties( TestPlyFilter PROPERTIES
        WIN32_EXECUTABLE False
    )
endif()

add_test( NAME TestPlyFilter COMMAND TestPlyFilter )

add_executable( TestFileRoundTrip )

target_sources( TestFileRoundTrip
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/TestFileRoundTrip.cpp
        ${CMAKE_CURRENT_LIST_DIR}/TestFileRoundTrip.h
)

target_link_libraries( TestFileRoundTrip
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

if ( WIN32 )
    set_target_properties( TestFileRoundTrip PROPERTIES
        WIN32_EXECUTABLE False
    )
endif()

add_test( NAME TestFileRoundTrip COMMAND TestFileRoundTrip )

if ( OPTION_USE_SHAPE_LIB )
    add_executable( TestShpFilter )

    target_sources( TestShpFilter
        PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/TestShpFilter.cpp
            ${CMAKE_CURRENT_LIST_DIR}/TestShpFilter.h
            ${CMAKE_CURRENT_LIST_DIR}/TestData.qrc
    )

    target_link_libraries( TestShpFilter
        PRIVATE
            QCC_IO_LIB
            Qt6::Test
    )

    if ( WIN32 )
        set_target_properties( TestShpFilter PROPERTIES
            WIN32_EXECUTABLE False
        )
    endif()

    add_test( NAME TestShpFilter COMMAND TestShpFilter )
endif()



