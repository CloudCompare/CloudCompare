# For private plugins, this is a shortcut to add all cpp, h, and ui files without listing them
# in their own CMakeLists.txt files.
#
# It will look in the plugin's top-level directory and in the src, include, and ui directories.
#
# This is provided for convenience, but you should understand GLOB and its benefits and drawbacks.
#  See: https://cmake.org/cmake/help/latest/command/file.html#filesystem
#
# To use it, simply add the following line to your plugin's top-level CMakeList.txt file:
#	include( FileGLOBBER )

file( GLOB
	PLUGIN_HEADERS
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

file( GLOB
	PLUGIN_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

file( GLOB
	PLUGIN_UI_FILES
		${CMAKE_CURRENT_SOURCE_DIR}/*.ui
		${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui
)


target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources( ${PROJECT_NAME}
   PRIVATE
	  ${PLUGIN_HEADERS}
	  ${PLUGIN_SOURCES}
	  ${PLUGIN_UI_FILES}
)

unset( PLUGIN_HEADERS )
unset( PLUGIN_SOURCES )
unset( PLUGIN_UI_FILES )
