cmake_minimum_required( VERSION 3.0 )

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_PUZZLER "Check to install the puzzler plugin" OFF )

if ( PLUGIN_PUZZLER )
	# Name the plugin
    project( QPUZZLER )
    
    # load any subdirectories (see qAdditionalIO for an example)
    # add_subdirectory( LIB1 )
    
    include( ../../CMakePluginTpl.cmake )
    
    # set dependencies to necessary libraries (see qPCV for an example)
    # target_link_libraries( ${PROJECT_NAME} LIB1 )
    # include_directories( ${LIB1_INCLUDE_DIR} )
endif()
