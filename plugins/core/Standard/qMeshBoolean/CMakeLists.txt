option( PLUGIN_STANDARD_QMESH_BOOLEAN "Install qMeshBoolean plugin" OFF )

# CloudCompare Mesh CSG operations plugin (based on the libigl library)
if( PLUGIN_STANDARD_QMESH_BOOLEAN )
	project( QMESH_BOOLEAN_PLUGIN )
	
	AddPlugin( NAME ${PROJECT_NAME} )

	# libIGL is needed to build this plugin
	include( cmake/LibIGLSupport.cmake NO_POLICY_SCOPE )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
		
	target_link_libIGL( ${PROJECT_NAME} )
endif()
