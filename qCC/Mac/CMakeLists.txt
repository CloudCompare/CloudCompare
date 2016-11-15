# Put together our App bundle on Mac OS X
if( APPLE )
   # this will install all the Qt libs and the necessary plugins
   install( CODE "execute_process( COMMAND \"${MACDEPLOYQT}\" \"${CLOUDCOMPARE_MAC_BASE_DIR}\" -verbose=1 )" COMPONENT Runtime )
   
   # install icons
   install( FILES ../images/icon/cc_icon.icns DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )
endif( APPLE )
