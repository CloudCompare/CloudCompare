cmake_minimum_required(VERSION 3.11) # FetchContent requires CMake 3.11

option( PLUGIN_STANDARD_QCHAISCRIPTING "Check to install ChaiScripting plugin" OFF )

if ( PLUGIN_STANDARD_QCHAISCRIPTING )
	
    # Name the plugin
    project( QCHAISCRIPTING )
    AddPlugin( NAME ${PROJECT_NAME} )
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
    
    target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)

    target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ChaiScriptingPlugin.cpp
)

endif()
