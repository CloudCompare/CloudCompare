# Put together our App bundle on Mac OS X
if( APPLE )
   # this will install all the Qt libs and the necessary plugins
   install( CODE "execute_process( COMMAND \"${MACDEPLOYQT}\" \"${CLOUDCOMPARE_MAC_BASE_DIR}\" -verbose=1 )" COMPONENT Runtime )
   
   # install icons
   install( FILES ../images/icon/cc_icon.icns DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Resources COMPONENT Runtime )   
   
   install( CODE "execute_process( COMMAND \"${CMAKE_SOURCE_DIR}/mac_scripts/delete_rpaths.sh\"
       \"${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/MacOS/${PROJECT_NAME}\"
       \"${CLOUDCOMPARE_MAC_FRAMEWORK_DIR}\"
       \"${CLOUDCOMPARE_MAC_PLUGIN_DIR}\" )" COMPONENT Runtime )
endif( APPLE )
