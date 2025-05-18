# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_STANDARD_QE57_IMAGE_EXTRACTOR "Install E57 Image Extractor plugin to add the ability to extract all 2d images from the E57 file" OFF )

if ( PLUGIN_STANDARD_QE57_IMAGE_EXTRACTOR )
	project( QE57ImageExtractor )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( include )
	add_subdirectory( src )

	# Ensure qE57_IO plugin is included as a dependency
	if ( NOT PLUGIN_IO_QE57 )
		message(FATAL_ERROR "E57ImageExtractor requires qE57_IO plugin. Please enable PLUGIN_IO_QE57 option. Run cmake with -DPLUGIN_IO_QE57=ON")
	endif()

	# Link against the qE57_IO plugin
	target_link_libraries( ${PROJECT_NAME} QE57_IO_PLUGIN )

	# Include directories from qE57_IO plugin if necessary
	include_directories(${CMAKE_SOURCE_DIR}/qE57_IO_PLUGIN/include)
	include_directories(${CMAKE_SOURCE_DIR}/qE57_IO_PLUGIN/extern/libE57Format/include)

endif()