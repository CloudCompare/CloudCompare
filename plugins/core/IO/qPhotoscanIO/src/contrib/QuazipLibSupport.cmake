# ------------------------------------------------------------------------------
# Quazip Lib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.0)

# Use system zlib on unix and Qt ZLIB on Windows
IF(UNIX OR MINGW)
    find_package(ZLIB REQUIRED)
ELSE()

    set( ZLIB_INCLUDE_DIRS "" CACHE PATH "zlib include directory" )
    set( ZLIB_LIBRARIES "" CACHE FILEPATH "zlib static library file" )

	if ( NOT ZLIB_INCLUDE_DIRS )
		message( SEND_ERROR "No zlib include dir specified (ZLIB_INCLUDE_DIRS)" )
	endif()

ENDIF()

# need Qt's XML package
#find_package(Qt5Xml REQUIRED)

project( quazip_static )

set(LOCAL_PATH src/contrib/quazip-0.7.3/quazip)
set(GLOBAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${LOCAL_PATH})

# set all include directories for in and out of source builds
include_directories(
	${GLOBAL_PATH}
	${ZLIB_INCLUDE_DIRS}
)

file(GLOB source_list ${GLOBAL_PATH}/*.c ${GLOBAL_PATH}/*.cpp)
file(GLOB header_list ${GLOBAL_PATH}/*.h)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

# Zlib
target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})
# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core)

target_compile_definitions( ${PROJECT_NAME} PRIVATE QUAZIP_STATIC )

# Link project with quazip library
function( target_link_QUAZIP ) # 1 argument: ARGV0 = project name

	if (ZLIB_INCLUDE_DIRS)

		include_directories( ${ZLIB_INCLUDE_DIRS} )

		target_compile_definitions( ${ARGV0} PRIVATE QUAZIP_STATIC )
		target_link_libraries( ${ARGV0} quazip_static )

	endif()

endfunction()
