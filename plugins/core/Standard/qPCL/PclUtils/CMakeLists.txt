project( QPCL_PLUGIN_UTILS_LIB )

find_package( PCL 1.9 REQUIRED )

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
else()
	
	# you'll need to point to pkg-config exectuable and make CMAKE_PREFIX_PATH point to the lz4 'installed' folder
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LZ4 REQUIRED liblz4)
    include_directories(${LZ4_INCLUDE_DIRS})

endif()
