cmake_minimum_required(VERSION 3.0)

option( PLUGIN_GL_QSSAO "Check to install qSSAO plugin" OFF )

# CloudCompare 'SSAO' plugin
if (PLUGIN_GL_QSSAO)
    add_subdirectory (Randomkit)
    
    project( QSSAO_GL_PLUGIN )
    
	include( CMakePolicies NO_POLICY_SCOPE )

    set( CC_SHADER_FOLDER SSAO )
    include( ../../../CMakePluginTpl.cmake )
    
    target_link_libraries( ${PROJECT_NAME} QSSAO_RANDOMKIT_LIB )
    include_directories( ${QSSAO_RANDOMKIT_LIB_SOURCE_DIR} )
endif()
