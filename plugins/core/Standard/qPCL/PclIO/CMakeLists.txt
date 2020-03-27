cmake_minimum_required(VERSION 3.0)

#CloudCompare PCL I/O plugin (for PCD files)
project( QPCL_IO_PLUGIN )

include ( CMakePolicies NO_POLICY_SCOPE )

include_directories( ${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/filters )
include_directories( ${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/utils )
include_directories( ${PCL_INCLUDE_DIRS} )

set( CC_IS_IO_PLUGIN 1 )

include( ../../../../CMakePluginTpl.cmake )

target_link_libraries(${PROJECT_NAME} QPCL_PLUGIN_UTILS_LIB)
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#define the PCL_VER_1_6_OR_OLDER preprocessor to compile qPclIO with older versions of PCL
if ( PCL_VERSION VERSION_LESS  1.7 ) # VERSION_GREATER Works just like "greater or equal"
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS PCL_VER_1_6_OR_OLDER )
endif()

#for ccViewer, we also have to copy the PCL dlls! (for CloudCompare they are already copied by qPCL
if( ${OPTION_BUILD_CCVIEWER} )

	#import PCL dlls (if any, WIN32 only)
	include( ../ExportPCLDlls.cmake )
	export_PCL_dlls( ${CCVIEWER_DEST_FOLDER} )

endif()
