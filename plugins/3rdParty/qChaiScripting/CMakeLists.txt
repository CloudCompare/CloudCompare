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
    else()
        add_definitions(-Wall -Wextra -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wcast-qual -Wunused -Woverloaded-virtual  -pedantic ${CPP11_FLAG})
        if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
          add_definitions(-Weverything -Wno-c++98-compat-pedantic  -Wno-c++98-compat -Wno-documentation -Wno-switch-enum -Wno-weak-vtables -Wno-missing-prototypes -Wno-padded -Wno-missing-noreturn -Wno-exit-time-destructors -Wno-documentation-unknown-command)
        else()
          add_definitions(-Wnoexcept)
        endif()

        if(APPLE)
          add_definitions(-Wno-sign-compare)
        endif()
    endif()
    include( ../../CMakePluginTpl.cmake )
endif()
