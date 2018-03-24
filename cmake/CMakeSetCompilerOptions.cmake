if( UNIX OR MINGW )
    # You need a c++11 Compiler to build CC
    # When we require cmake 3.1, we can use a cleaner method:
    #   CXX_STANDARD & CXX_STANDARD_REQUIRED
    #   https://cmake.org/cmake/help/v3.1/prop_tgt/CXX_STANDARD.html
    include(CheckCXXCompilerFlag)
    
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    
    if (NOT COMPILER_SUPPORTS_CXX11)
        message(ERROR "Your compiler does not support C++11")
    endif()
    
    set( CXX11_FLAG "-std=c++11")
    
    # MinGW doesn't use fPIC
    if( UNIX )
        set( FPIC_FLAG  "-fPIC")
    endif()
    
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG} ${FPIC_FLAG}")
    set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FPIC_FLAG}")
elseif( MSVC )
    add_definitions(-DNOMINMAX -D_CRT_SECURE_NO_WARNINGS -D__STDC_LIMIT_MACROS)

    OPTION( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
       set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()

    #disable SECURE_SCL (see http://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/)
	set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /D _SECURE_SCL=0" ) # disable checked iterators

    #use VLD for mem leak checking
    OPTION( OPTION_USE_VISUAL_LEAK_DETECTOR "Check to activate compilation (in debug) with Visual Leak Detector" OFF )
    if( ${OPTION_USE_VISUAL_LEAK_DETECTOR} )
		set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /D USE_VLD" )
    endif()
endif()
