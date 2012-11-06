cmake_minimum_required(VERSION 2.6)

project(dist_test)

set(XIOT_INCLUDE_DIR "" CACHE PATH "Include path for XIOT")
set(XIOT_LIBRARY "" CACHE FILEPATH  "Location of XIOT library" )
set(OPENFI_LIBRARY "" CACHE FILEPATH  "Location of openFI library" )

include_directories(${XIOT_INCLUDE_DIR})

#simpleTest
add_executable (dist_test dist_test.cpp)
target_link_libraries(dist_test ${XIOT_LIBRARY} ${OPENFI_LIBRARY})

