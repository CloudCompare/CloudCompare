cmake_minimum_required( VERSION 3.0 )

# REPLACE ALL 'ExamplePlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

option( INSTALL_QEXAMPLE_PLUGIN "Check to install qExamplePlugin" OFF )

# CloudCompare example for standard plugins
if ( INSTALL_QEXAMPLE_PLUGIN )
    project( QEXAMPLE_PLUGIN )
    
    # load files in subdirectories (see qAdditionalIO for an example)
    # add_subdirectory( LIB1 )
    
    include( ../CMakePluginTpl.cmake )
    
    # set dependencies to necessary libraries (see qPCV for an example)
    # target_link_libraries( ${PROJECT_NAME} LIB1 )
    # include_directories( ${LIB1_INCLUDE_DIR} )
endif()
