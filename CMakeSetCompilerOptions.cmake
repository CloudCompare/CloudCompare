# cmake should be smart enough to automatically append
# the NDEBUG definition or _DEBUG in Release/Debug modes.
# thus there should no need to force them somehow
if( UNIX )
    add_definitions("-fPIC")    # is the easier way to add the flag. cmake will take care of everything

    # You need a c++11 Compiler to build CC
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
        set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    else()
        message(ERROR "Your compiler does not support C++11")
    endif()
endif()

if( MSVC )
    add_definitions(-DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)

    OPTION( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" OFF )
    if( ${OPTION_MP_BUILD} )
       set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}\ /MP)
    endif()

    #disable SECURE_SCL (see http://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/)
    list( APPEND CCMAKE_CXX_FLAGS_RELEASE _SECURE_SCL=0 ) # disable checked iterators

    #use VLD for mem leak checking
    OPTION( OPTION_USE_VISUAL_LEAK_DETECTOR "Check to activate compilation (in debug) with Visual Leak Detector" OFF )
    if( ${OPTION_USE_VISUAL_LEAK_DETECTOR} )
       list( APPEND CCMAKE_CXX_FLAGS_DEBUG USE_VLD )
    endif()
endif(MSVC)
