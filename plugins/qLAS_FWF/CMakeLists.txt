cmake_minimum_required(VERSION 3.0)

# Windows only for now
if (WIN32)

	option( INSTALL_QLAS_FWF_IO_PLUGIN "Check to install qLAS FWF I/O plugin" OFF )
	if ( INSTALL_QLAS_FWF_IO_PLUGIN )

		#CloudCompare LAS FWF (i.e. version >= 1.3) I/O plugin
		project( QLAS_FWF_IO_PLUGIN )
		
		include( LASLibSupport.cmake )

		set( CC_PLUGIN_CUSTOM_HEADER_LIST Filter/LASFWFFilter.h )
		set( CC_PLUGIN_CUSTOM_SOURCE_LIST Filter/LASFWFFilter.cpp )
			
		#we need the saveLASFileDlg.ui file
		set( CC_PLUGIN_CUSTOM_UI_LIST ${QCC_IO_LIB_SOURCE_DIR}/ui_templates/saveLASFileDlg.ui )

		set( CC_IS_IO_PLUGIN 1 )

		include( ../CMakePluginTpl.cmake )
		
		target_link_LASLib( ${PROJECT_NAME} )

	endif()

endif()