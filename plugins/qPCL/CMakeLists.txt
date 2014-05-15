cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()

option( INSTALL_QPCL_PLUGIN "Check to install qPCL plugin" OFF )

if (INSTALL_QPCL_PLUGIN)

#CloudCompare 'PCL' bridge plugin
project( QPCL_PLUGIN )

add_subdirectory(PclUtils)
add_subdirectory(doc)

#include_directories(pcl)
#include_directories(filters)
#include_directories(filters/dialogs)
include_directories( ${QPCL_PLUGIN_UTILS_LIB_SOURCE_DIR}/filters )
include_directories( ${QPCL_PLUGIN_UTILS_LIB_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../) #the plugin dir



#requires PCL
find_package(PCL REQUIRED)

include( ../CMakePluginTpl.cmake )

target_link_libraries(${PROJECT_NAME} QPCL_PLUGIN_UTILS_LIB)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#import PCL dlls (if any)
if (WIN32 AND PCL_DIR)

# first of all check if is setted as "${PCL_DIR}" or "${PCL_DIR}/cmake". Not sure way but it happen on my win7 system

get_filename_component(last_dir ${PCL_DIR} NAME) # get the last line of ${PCL_DIR}
if(last_dir STREQUAL "cmake")
	get_filename_component(PCL_DIR ${PCL_DIR} PATH) #trim PCL_DIR path if needed
endif()

# now find pcl libs we need
file( GLOB pcl_release_dlls ${PCL_DIR}/bin/*${PCL_RELEASE_SUFFIX}.dll  )
file( GLOB pcl_debug_dlls ${PCL_DIR}/bin/*${PCL_DEBUG_SUFFIX}.dll  )
	


	#release DLLs
	file( GLOB pcl_release_dlls ${PCL_DIR}/bin/*${PCL_RELEASE_SUFFIX}.dll  )
	foreach( filename ${pcl_release_dlls} )
		if( CMAKE_CONFIGURATION_TYPES )
			install( FILES ${filename} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} )
		else()
			install( FILES ${filename} CONFIGURATIONS Release DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} )
			install( FILES ${filename} CONFIGURATIONS RelWithDebInfo DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}_withDebInfo )
		endif()
	endforeach()

	#debug DLLs
	if( CMAKE_CONFIGURATION_TYPES )
		file( GLOB pcl_debug_dlls ${PCL_DIR}/bin/*${PCL_DEBUG_SUFFIX}.dll  )
		foreach( filename ${pcl_debug_dlls} )
			install( FILES ${filename} CONFIGURATIONS Debug DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}_debug )
		endforeach()
	endif()

endif()


endif()


