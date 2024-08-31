project( QPCL_IO_PLUGIN )

AddPlugin( NAME ${PROJECT_NAME} TYPE io )

add_subdirectory( include )
add_subdirectory( src )
add_subdirectory( ui )

target_link_libraries( ${PROJECT_NAME} QPCL_PLUGIN_UTILS_LIB )

#for ccViewer, we also have to copy the PCL dlls! (for CloudCompare they are already copied by qPCL
if( ${OPTION_BUILD_CCVIEWER} )

	#import PCL dlls (if any, WIN32 only)
	include( ../cmake/ExportPCLDlls.cmake )
	export_PCL_dlls( "${CCVIEWER_DEST_FOLDER}" )

endif()
