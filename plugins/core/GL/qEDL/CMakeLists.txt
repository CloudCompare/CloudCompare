cmake_minimum_required(VERSION 3.0)

option( PLUGIN_GL_QEDL "Check to install qEDL plugin" OFF )

# CloudCompare 'EDL' plugin
if (PLUGIN_GL_QEDL)
    project( QEDL_GL_PLUGIN )
    
	include( CMakePolicies NO_POLICY_SCOPE )

	include_directories( src )
	
	add_subdirectory( src )
	
    set( CC_SHADER_FOLDER EDL )
    include( ../../../CMakePluginTpl.cmake )
endif()
