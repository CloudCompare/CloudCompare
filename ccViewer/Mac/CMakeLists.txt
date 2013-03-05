# Put together our App bundle on Mac OS X
if( APPLE )
   # install support libs
   # FIXME this shouldn't be done manually - not sure how to do this properly
   list( APPEND SUPPORT_LIB_NAMES libCC_DLL )
   list( APPEND SUPPORT_LIB_NAMES libQCC_DB_DLL )
   
   foreach( supportLib ${SUPPORT_LIB_NAMES} )
      set( LIB_NAME ${supportLib}${CMAKE_SHARED_LIBRARY_SUFFIX} )
      install( FILES ${CMAKE_INSTALL_PREFIX}/lib/${LIB_NAME} DESTINATION ${MAC_BUNDLE_MACOS_DIR} COMPONENT Runtime )
   endforeach()

   # install qt.conf
   install( CODE "
      file( WRITE ${MAC_BUNDLE_RESOURCES_DIR}/qt.conf \"\" )
      " COMPONENT Runtime )

   set( DIRS ${EXTERNAL_LIBS_LIBRARIES} )

   # call fixup_bundles to change paths using otools
   install( CODE "
      include( BundleUtilities )
      fixup_bundle( ${CMAKE_INSTALL_PREFIX}/${ccViewer_dest_release}/${MAC_APP_NAME} \"\" \"${DIRS}\" COMPONENT Runtime ) 
      ")
endif( APPLE )