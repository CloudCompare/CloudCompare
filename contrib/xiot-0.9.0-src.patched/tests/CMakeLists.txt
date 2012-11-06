cmake_minimum_required(VERSION 2.6)

include_directories(. ${PROJECT_SOURCE_DIR}/include)

#simpleTest
add_executable (simpleTest simpleTest.cpp Argument_helper.cpp)
target_link_libraries(simpleTest xiot)

#shapeCounter
add_executable (shapeCounter shapeCounter.cpp Argument_helper.cpp)
target_link_libraries(shapeCounter xiot)

#FITest
add_executable (fitest fitest.cpp Argument_helper.cpp)
target_link_libraries(fitest xiot)

#IOTest
add_executable (ioTest IOTest.cpp Argument_helper.cpp)
target_link_libraries(ioTest xiot)

#ParserPerformance
add_executable (parserPerformance parserPerformance.cpp Argument_helper.cpp)
target_link_libraries(parserPerformance xiot)


#createEventLog
add_executable (createEventLog createEventLog.cpp Argument_helper.cpp X3DLogNodeHandler.cpp X3DLogNodeHandler.h)
target_link_libraries(createEventLog xiot)


install(TARGETS simpleTest shapeCounter fitest createEventLog RUNTIME DESTINATION bin)