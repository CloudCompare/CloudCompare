# Put together our App bundle on macOS
if( APPLE )
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		INSTALL_RPATH "@executable_path/../Frameworks"
		MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/CloudCompare.plist
		MACOSX_BUNDLE_ICON_FILE cc_icon.icns
		MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
		MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_VERSION}"
		MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}" )

	set( CLOUDCOMPARE_MAC_BASE_DIR ${CMAKE_INSTALL_PREFIX}/${CLOUDCOMPARE_DEST_FOLDER}/CloudCompare.app  CACHE INTERNAL "CloudCompare bundle dir")
	set( CLOUDCOMPARE_MAC_FRAMEWORK_DIR ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Frameworks CACHE INTERNAL "CC framework dir" )
	set( CLOUDCOMPARE_MAC_PLUGIN_DIR ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/PlugIns/ccPlugins CACHE INTERNAL "CC plugin dir" )
	
	# this will install all the Qt libs and the necessary plugins
	install( CODE "execute_process( COMMAND \"${MACDEPLOYQT}\" \"${CLOUDCOMPARE_MAC_BASE_DIR}\" -verbose=1 )" COMPONENT Runtime )
	
	# install icons
	install( FILES cc_icon.icns DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )   
endif( APPLE )
