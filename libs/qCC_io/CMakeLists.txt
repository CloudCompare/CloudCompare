include( CMakePolicies NO_POLICY_SCOPE )

project( QCC_IO_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

target_link_libraries( ${PROJECT_NAME}
	QCC_DB_LIB
)

# contrib. libraries support
target_link_contrib( ${PROJECT_NAME} )

# Add custom preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_IO_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
	CXX_VISIBILITY_PRESET hidden
)

if ( BUILD_TESTING )
	add_subdirectory( test ) 
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )

