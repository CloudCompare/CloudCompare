project( QPCL_PLUGIN_UTILS_LIB )

find_package( PCL 1.15 REQUIRED )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( filters )
add_subdirectory( utils )

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${PCL_INCLUDE_DIRS}
)

target_link_directories( ${PROJECT_NAME}
		PUBLIC
			${PCL_LIBRARY_DIRS}
	)

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		CCPluginAPI
		QCC_GL_LIB
		${PCL_LIBRARIES}
)

if( NOT WIN32 )

	if( OpenMP_CXX_FOUND )
		message( STATUS "Build PCL WITH OpenMPdd")
		target_link_libraries( ${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX )
	endif()
else()

	# You'll need to point to pkg-config exectuable and make CMAKE_PREFIX_PATH point to the lz4 'installed' folder
	find_package( PkgConfig REQUIRED )
	pkg_check_modules( LZ4 REQUIRED liblz4 )
	include_directories( ${LZ4_INCLUDE_DIRS} )

endif()
