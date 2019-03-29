cmake_minimum_required(VERSION 3.0)

option( PLUGIN_STANDARD_QCANUPO "Check to install qCanupo plugin" OFF )

if (PLUGIN_STANDARD_QCANUPO)

	#CloudCompare 'Canupo' plugin
	project( QCANUPO_PLUGIN )
	
	set( dlib_DIR "" CACHE PATH "DLib library root directory" )
	if ( NOT dlib_DIR )
		message( SEND_ERROR "No DLib library root specified (dlib_DIR)" )
	else()
		find_package(dlib REQUIRED)
		include_directories( ${dlib_INCLUDE_DIRS} )
	endif()
	
	include( ../../../CMakePluginTpl.cmake )
	target_link_libraries(${PROJECT_NAME} ${dlib_LIBRARIES})
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "plugins")
	option( COMPILE_QCANUPO_WITH_PRIVATE_PARTS "Check to compile qCanupo plugin with private methods (CNRS/UEB only)" OFF )
	if ( COMPILE_QCANUPO_WITH_PRIVATE_PARTS )
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS COMPILE_PRIVATE_CANUPO )
	endif()

endif()
