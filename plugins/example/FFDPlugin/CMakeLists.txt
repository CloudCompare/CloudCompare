# CloudCompare FFD (Free Form Deformation) Plugin
# Allows non-rigid transformation of point clouds using a lattice structure

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_FFD_STANDARD "Install FFD (Free Form Deformation) plugin" OFF )

if ( PLUGIN_FFD_STANDARD )
	project( FFDPlugin )

	option( FFD_DEBUG_LOGGING "Enable FFD plugin debug logging to stderr" OFF )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )

	if ( FFD_DEBUG_LOGGING )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE FFD_DEBUG_ENABLED )
		message( STATUS "FFD plugin: debug logging ENABLED" )
	endif()
	
	# Link against the necessary libraries for math operations
	# target_link_libraries( ${PROJECT_NAME} )
endif()
