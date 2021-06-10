cmake_minimum_required(VERSION 3.11) # FetchContent requires CMake 3.11

option( PLUGIN_STANDARD_QCHAISCRIPTING "Check to install ChaiScripting plugin" OFF )

if ( PLUGIN_STANDARD_QCHAISCRIPTING )
	
    # Name the plugin
    project( QCHAISCRIPTING )
    
    add_subdirectory(cmake)
    include_directories(${chaiscript_SOURCE_DIR}/include)
    include_directories( codeEditor )
    include_directories( bootstrap )

	add_subdirectory( codeEditor )
    add_subdirectory( ui )
    add_subdirectory( bootstrap )
    if(MSVC)
        add_definitions(/bigobj) # REQUIRED
    endif()
    include( ../../CMakePluginTpl.cmake )
endif()
