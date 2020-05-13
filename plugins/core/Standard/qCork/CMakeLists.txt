option( PLUGIN_STANDARD_QCORK "Install qCORK plugin" OFF )

# CloudCompare Mesh CSG operations plugin (based on the Cork library)
if( PLUGIN_STANDARD_QCORK )
	project( QCORK_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )

	# Cork lib and MPIR are needed to build this plugin
	include( cmake/CorkSupport.cmake NO_POLICY_SCOPE )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
		
	target_link_cork( ${PROJECT_NAME} )
endif()
