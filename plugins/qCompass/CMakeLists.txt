cmake_minimum_required(VERSION 3.0)

#REPLACE ALL 'DUMMY' OCCURENCES BY YOUR PLUGIN NAME
#AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

option( INSTALL_QCC_COMPASS_PLUGIN "Check to install compass plugin" ON )

# CloudCompare 'compass' plugin
if (INSTALL_QCC_COMPASS_PLUGIN)
    #project( CC_COMPASS_PLUGIN )
    
    #load necessary libraries (see qPCV for an example)
    #add_subdirectory (LIB1)
    
    #if the plugin is an 'OpenGL filter', uncomment the line below
    #set( CC_OPENGL_FILTER ON BOOL)

    project( CC_COMPASS_PLUGIN )

	#As we are going to re-use several components of the main qCC project
	#we define its sources path relatively to the plugin's dir
	set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)
	
	#include the whole CC directory (easier)
    include_directories( ${CloudCompare_SOURCE_DIR} )
	
	#we need ccOverlay classes
	file( GLOB CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.h )
	file( GLOB CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccOverlayDialog*.cpp )

	#we also need picking hub classes
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingListener.h )
	list( APPEND  CC_PLUGIN_CUSTOM_HEADER_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.h )	
	list( APPEND  CC_PLUGIN_CUSTOM_SOURCE_LIST ${CloudCompare_SOURCE_DIR}/ccPickingHub.cpp )

	#As we are going to re-use several components of the main qCC project
    #we define its sources path relatively to the plugin's dir
    #include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)
    #not sure this is the correcrt way of doing this but adding in 
    #qCC build file so it knows where the ui_*.h files are
    set( CC_PLUGIN_CUSTOM_UI_LIST	${CMAKE_CURRENT_SOURCE_DIR}/ui/compassDlg.ui )

	
    #add IO support
    #include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
    #target_link_libraries( ${PROJECT_NAME} QCC_IO_LIB )

    #set dependencies to necessary libraries (see qPCV for an example)
    #target_link_libraries( ${PROJECT_NAME} LIB1 )
    #include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC )

	include( ../CMakePluginTpl.cmake  )

endif()
