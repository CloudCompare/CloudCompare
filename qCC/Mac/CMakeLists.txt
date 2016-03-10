# Put together our App bundle on Mac OS X
if( APPLE )
   # this will install all the Qt libs and the necessary plugins
   install( CODE "execute_process( COMMAND \"${MACDEPLOYQT}\" \"${CLOUDCOMPARE_MAC_BASE_DIR}\" -verbose=1 )" COMPONENT Runtime )
   
   # install icons
   install( FILES ../images/icon/cc_icon.icns DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )   
   
   set( PLUGINS ${QT_PLUGINS} ${CLOUDCOMPARE_PLUGINS} )
       
   # call fixup_bundles to change paths using otools
   install( CODE "
      include( BundleUtilities )
      fixup_bundle( ${CLOUDCOMPARE_MAC_BASE_DIR} \"${PLUGINS}\" \"${DIRS}\" COMPONENT Runtime ) 
      ")
endif( APPLE )
