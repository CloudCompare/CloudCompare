find_package( Qt5Test REQUIRED )

if ( OPTION_USE_SHAPE_LIB )
    add_executable( TestShpFilter )

    target_sources( TestShpFilter
        PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/TestShpFilter.cpp
            ${CMAKE_CURRENT_LIST_DIR}/TestShpFilter.h
            ${CMAKE_CURRENT_LIST_DIR}/TestData.qrc
    )

    target_link_libraries( TestShpFilter
        QCC_IO_LIB
        Qt5::Test
    )

    if ( WIN32 )
        set_target_properties( TestShpFilter PROPERTIES
            WIN32_EXECUTABLE False
        )
    endif()

    add_test( NAME TestShpFilter COMMAND TestShpFilter )
endif()



