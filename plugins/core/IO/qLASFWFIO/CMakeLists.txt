# Windows only for now
if (WIN32)

	option( PLUGIN_IO_QLAS_FWF "Check to install qLAS FWF I/O plugin" OFF )
	
	if ( PLUGIN_IO_QLAS_FWF )

		#CloudCompare LAS FWF (i.e. version >= 1.3) I/O plugin
		project( QLAS_FWF_IO_PLUGIN )
		
        AddPlugin( NAME ${PROJECT_NAME} TYPE io )

		include( LASLibSupport.cmake )

		set( CC_PLUGIN_CUSTOM_HEADER_LIST Filter/LASFWFFilter.h )
		set( CC_PLUGIN_CUSTOM_SOURCE_LIST Filter/LASFWFFilter.cpp )
			
		#we need the saveLASFileDlg.ui file
		set( CC_PLUGIN_CUSTOM_UI_LIST ../qPDALIO/ui/saveLASFileDlg.ui )

        include( CMakePluginTemplate )
		
		target_link_LASLib( ${PROJECT_NAME} )

	endif()

endif()
