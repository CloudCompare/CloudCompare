cmake_minimum_required(VERSION 2.8)

option( INSTALL_QPCL_PLUGIN "Check to install qPCL plugin" OFF )

if (INSTALL_QPCL_PLUGIN)

#CloudCompare 'PCL' bridge plugin
project( QPCL_PLUGIN_DLL )

add_subdirectory(PclUtils)
add_subdirectory(doc)

#include_directories(pcl)
#include_directories(filters)
#include_directories(filters/dialogs)
include_directories( ${PclUtils_SOURCE_DIR}/filters )
include_directories( ${PclUtils_BINARY_DIR} )



#requires PCL
find_package(PCL REQUIRED)

include( ../CMakePluginTpl.cmake )

target_link_libraries(${PROJECT_NAME} PclUtils)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#import PCL dlls
if (WIN32 AND PCL_DIR)

file( GLOB pcl_release_dlls ${PCL_DIR}/bin/*${PCL_RELEASE_SUFFIX}.dll  )
file( GLOB pcl_debug_dlls ${PCL_DIR}/bin/*${PCL_DEBUG_SUFFIX}.dll  )
	
#release DLLs
foreach( filename ${pcl_release_dlls} )
	install_ext( FILES ${filename} ${qCC_dest_release} )
endforeach()

#debug DLLs
foreach( filename ${pcl_debug_dlls} )
	install_ext( FILES ${filename} 0 ${qCC_dest_debug} )
endforeach()

endif()


endif()


