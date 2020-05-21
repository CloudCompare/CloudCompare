project( QPCL_PLUGIN_UTILS_LIB )

find_package( Qt5 COMPONENTS Widgets REQUIRED )

find_package( PCL 1.7 REQUIRED )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( filters )
add_subdirectory( utils )

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${PCL_INCLUDE_DIRS}
)

if( CMAKE_VERSION VERSION_LESS 3.13 )
	link_directories( ${PCL_LIBRARY_DIRS} )
else()
	target_link_directories( ${PROJECT_NAME}
		PUBLIC
		    ${PCL_LIBRARY_DIRS}
	)
endif()

target_link_libraries( ${PROJECT_NAME}
	CCPluginAPI
	QCC_GL_LIB
	${PCL_LIBRARIES}
)

if( NOT WIN32 )
	find_package( OpenMP )

	if( OpenMP_CXX_FOUND )
		target_link_libraries( ${PROJECT_NAME} OpenMP::OpenMP_CXX )
	endif()
endif()
