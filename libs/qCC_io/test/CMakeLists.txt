find_package(Qt6 REQUIRED COMPONENTS Test)

# PLY Filter tests
add_executable(TestPlyFilter
    TestPlyFilter.cpp
)

target_link_libraries(TestPlyFilter
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

add_test( NAME TestPlyFilter COMMAND TestPlyFilter )

# ASCII open dialog tests
add_executable( TestAsciiOpenDlg
    TestAsciiOpenDlg.cpp
)

target_link_libraries( TestAsciiOpenDlg
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

add_test( NAME TestAsciiOpenDlg COMMAND TestAsciiOpenDlg )

# File Round Trip tests
add_executable( TestFileRoundTrip
    TestFileRoundTrip.cpp
)

target_link_libraries(TestFileRoundTrip
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

add_test( NAME TestFileRoundTrip COMMAND TestFileRoundTrip )

# DepthMap Filter tests
add_executable( TestDepthMapFilter
    TestDepthMapFilter.cpp
)

target_link_libraries( TestDepthMapFilter
    PRIVATE
        QCC_IO_LIB
        Qt6::Test
)

add_test( NAME TestDepthMapFilter COMMAND TestDepthMapFilter )

# Shape files filter tests
if ( OPTION_USE_SHAPE_LIB )

    add_executable( TestShpFilter
        TestShpFilter.cpp
        TestData.qrc
    )

    target_link_libraries( TestShpFilter
        PRIVATE
            QCC_IO_LIB
            Qt6::Test
    )

    add_test( NAME TestShpFilter COMMAND TestShpFilter )

endif()



