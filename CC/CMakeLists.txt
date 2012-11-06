cmake_minimum_required(VERSION 2.8)

# to compile CCLib only! (CMake implicitely imposes to declare a project before anything...)
project( CC_DUMMY_PROJECT )

# CC_DLL only relies on 'triangle' library...
add_subdirectory (triangle)
# ... and QT!
set( DESIRED_QT_VERSION 4 )
find_package( Qt4 ${QT_VERSION} COMPONENTS QtCore REQUIRED )
if( NOT QT_FOUND )
    message( SEND_ERROR "Qt required, but not found with 'find_package()'" )
else()
    include( ${QT_USE_FILE} )
endif()

project( CC_DLL )

set( SAMPLE_PROJECT_VERSION_MAJOR 1 )
set( SAMPLE_PROJECT_VERSION_MINOR 0 )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories( ${triangle_SOURCE_DIR} )

file( GLOB header_list include/*.h)
file( GLOB source_list src/*.cpp src/*.h)

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} triangle )
target_link_libraries( ${PROJECT_NAME} ${QT_LIBRARIES} )

# Load advanced scripts
include( ../CMakeInclude.cmake )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL )
endif()

# Install (shared) library to specified destinations
if( UNIX )
	install_shared( CC_DLL lib lib ) #default destination: /usr/lib
else()
foreach( dest ${INSTALL_DESTINATIONS} )
	install_shared( CC_DLL ${dest} ${dest}_debug )
endforeach()
endif()
