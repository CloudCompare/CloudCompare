project( QCC_DB_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

add_subdirectory( extern/CCCoreLib EXCLUDE_FROM_ALL )

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
    	CCCoreLib
		Qt6::Widgets
		Qt6::OpenGL
		CC_FBO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD QT_NO_KEYWORDS )

if(OpenMP_CXX_FOUND)
    target_link_libraries( ${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX )
endif()

set_target_properties( ${PROJECT_NAME} PROPERTIES
	CXX_VISIBILITY_PRESET hidden
)

InstallSharedLibrary( TARGET CCCoreLib )
InstallSharedLibrary( TARGET ${PROJECT_NAME} )

include( cmake/InstallCGALDependencies.cmake )
