# Require C++14
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS NO )

# ccache
# https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program( CCACHE_PROGRAM ccache )

if ( CCACHE_PROGRAM )
    set( CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
    set( CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
endif()

if ( UNIX )
	set( CMAKE_POSITION_INDEPENDENT_CODE ON )	
elseif( MSVC )
    add_definitions(-DNOMINMAX -D_CRT_SECURE_NO_WARNINGS -D__STDC_LIMIT_MACROS)

    option( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
       set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()

    #disable SECURE_SCL (see http://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/)
	set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /D _SECURE_SCL=0" ) # disable checked iterators

    #use VLD for mem leak checking
    option( OPTION_USE_VISUAL_LEAK_DETECTOR "Check to activate compilation (in debug) with Visual Leak Detector" OFF )
    if( ${OPTION_USE_VISUAL_LEAK_DETECTOR} )
		set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /D USE_VLD" )
    endif()
endif()
