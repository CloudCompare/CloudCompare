find_package(Qt5Test REQUIRED)


QT5_ADD_RESOURCES( TestDataResources_SRCS TestData.qrc )

SET(CMAKE_WIN32_EXECUTABLE False)
set(TEST_LIBRARIES Qt5::Test Qt5::Core Qt5::WinMain CC_CORE_LIB QCC_DB_LIB QCC_IO_LIB)

SET(TestShpFilter_SRC TestShpFilter.cpp)
ADD_EXECUTABLE(TestShpFilter ${TestShpFilter_SRC} ${TestDataResources_SRCS})
TARGET_LINK_LIBRARIES(TestShpFilter ${TEST_LIBRARIES})
ADD_TEST(NAME TestShpFilter COMMAND TestShpFilter)

