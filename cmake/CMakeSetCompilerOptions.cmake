# Require C++17
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS NO )

# ccache
# https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program( CCACHE_PROGRAM ccache )

if ( CCACHE_PROGRAM )
	set( CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
	set( CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
endif()


add_compile_definitions(QT_DISABLE_DEPRECATED_UP_TO=0x050F00)

if ( UNIX )
	set( CMAKE_POSITION_INDEPENDENT_CODE ON )
	add_compile_options(-Wno-deprecated-declarations)

elseif( MSVC )

	# Enable MP build by default
	option( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" ON )
	if( OPTION_MP_BUILD )
		add_compile_options( /MP )
	endif()

	# We disable SECURE_SCL in Release builds
	# (see http://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/)
	add_compile_definitions( NOMINMAX
		_CRT_SECURE_NO_WARNINGS
		__STDC_LIMIT_MACROS
		$<$<CONFIG:Release>:_SECURE_SCL=0>
	)

	# Use VLD for mem leak checking
	option( OPTION_USE_VISUAL_LEAK_DETECTOR "Check to activate compilation (in debug) with Visual Leak Detector" OFF )
	if( ${OPTION_USE_VISUAL_LEAK_DETECTOR} )
		add_compile_definitions( $<$<CONFIG:Debug>:USE_VLD> )
	endif()
endif()
