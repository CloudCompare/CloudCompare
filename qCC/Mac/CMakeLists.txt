# Put together our App bundle on Mac OS X
if( APPLE )      
   # install imageformat plugins
	install_Qt_ImageFormats( ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Plugins QT_PLUGINS )

   # install support libs
   get_support_libs( SUPPORT_LIBS )
   install( FILES ${SUPPORT_LIBS} DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/MacOS COMPONENT Runtime )
   
   # install icons
   install( FILES ../images/icon/cc_icon.icns DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )   

   # install qt.conf
   install( CODE "
      file( WRITE ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources/qt.conf \"[Paths]\nPlugins = Plugins\" )
      " COMPONENT Runtime )
   
   set( DIRS ${EXTERNAL_LIBS_LIBRARIES} )
   set( PLUGINS ${QT_PLUGINS} ${CLOUDCOMPARE_PLUGINS} )

   if( ${OPTION_USE_LIBLAS} )
      set( DIRS ${DIRS} ${Boost_LIBRARY_DIRS} )
   endif()

   # call fixup_bundles to change paths using otools
   install( CODE "
      include( BundleUtilities )
      fixup_bundle( ${CLOUDCOMPARE_MAC_BASE_DIR} \"${PLUGINS}\" \"${DIRS}\" COMPONENT Runtime ) 
      ")
endif( APPLE )