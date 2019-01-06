find_package(Qt5Test REQUIRED)


QT5_ADD_RESOURCES(TestDataResources_SRCS TestData.qrc)

set(TEST_LIBRARIES Qt5::Test Qt5::Core CC_CORE_LIB QCC_DB_LIB QCC_IO_LIB)

if (WIN_32)
    SET(CMAKE_WIN32_EXECUTABLE False)
    set(TEST_LIBRARIES ${TEST_LIBRARIES} Qt5::WinMain)
endif()

if (OPTION_USE_SHAPE_LIB)
    SET(TestShpFilter_SRC TestShpFilter.cpp)
    ADD_EXECUTABLE(TestShpFilter ${TestShpFilter_SRC} ${TestDataResources_SRCS})
    TARGET_LINK_LIBRARIES(TestShpFilter ${TEST_LIBRARIES})
    set_property( TARGET TestShpFilter APPEND PROPERTY COMPILE_DEFINITIONS CC_SHP_SUPPORT )
    ADD_TEST(NAME TestShpFilter COMMAND TestShpFilter)
endif()



