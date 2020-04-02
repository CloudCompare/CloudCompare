# Put together our App bundle on macOS
if( APPLE )
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		INSTALL_RPATH "@executable_path/../Frameworks"
		MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/ccViewer.plist
		MACOSX_BUNDLE_ICON_FILE cc_viewer_icon.icns
		MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
		MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_VERSION}"
		MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}" )

	set( CCVIEWER_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CCVIEWER_DEST_FOLDER}/ccViewer.app CACHE INTERNAL "ccViewer bundle dir")
	set( CCVIEWER_MAC_FRAMEWORK_DIR ${CCVIEWER_MAC_BASE_DIR}/Contents/Frameworks CACHE INTERNAL "CCViewer framework dir" )
	set( CCVIEWER_MAC_PLUGIN_DIR ${CCVIEWER_MAC_BASE_DIR}/Contents/PlugIns/ccPlugins CACHE INTERNAL "CCViewer plugin dir" )

	# this will install all the Qt libs and the necessary plugins
	install( CODE "execute_process( COMMAND \"${MACDEPLOYQT}\" \"${CCVIEWER_MAC_BASE_DIR}\" -verbose=1 )" COMPONENT Runtime )
	
	# install icons
	install( FILES cc_viewer_icon.icns DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )

	# Export common shader files to all install destinations
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC/shaders/ColorRamp/color_ramp.frag DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
endif( APPLE )
