cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_STANDARD_QCHAISCRIPTING "Check to install ChaiScripting plugin" OFF )

if ( PLUGIN_STANDARD_QCHAISCRIPTING )
	
    # Name the plugin
    project( QCHAISCRIPTING )
    add_definitions(-DCHAISCRIPT_NO_THREADS) #avoiding a crash that happens when destroying chai with threads enabled
    
    add_subdirectory(cmake)
    include_directories(${chaiscript_SOURCE_DIR}/include)
    include_directories( src )
	add_subdirectory( src )
    add_subdirectory( ui )
    add_definitions(/bigobj) # REQUIRED
    
    include( ../../CMakePluginTpl.cmake )
endif()
