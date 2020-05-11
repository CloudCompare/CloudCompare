project( QPCL_PLUGIN_UTILS_LIB )

find_package( Qt5 COMPONENTS Widgets REQUIRED )

find_package( PCL 1.7 REQUIRED )

if( NOT WIN32 )
	find_package( OpenMP )

	if( OpenMP_CXX_FOUND )
		target_link_libraries( ${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX )
	endif()
endif()

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
    CCPluginAPI
    QCC_GL_LIB
    ${PCL_LIBRARIES}
    Qt5::Widgets
)
