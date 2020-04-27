### DEFAULT CC "STANDARD" PLUGIN CMAKE SCRIPT ###

include( CMakePolicies NO_POLICY_SCOPE )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

file( GLOB json_list *.json)
file( GLOB ui_list *.ui )
file( GLOB qrc_list *.qrc )
file( GLOB rc_list *.rc )

if ( CC_PLUGIN_CUSTOM_HEADER_LIST )
    list( APPEND header_list ${CC_PLUGIN_CUSTOM_HEADER_LIST} )
endif()

if ( CC_PLUGIN_CUSTOM_SOURCE_LIST )
    list( APPEND source_list ${CC_PLUGIN_CUSTOM_SOURCE_LIST} )
endif()

if (CC_PLUGIN_CUSTOM_UI_LIST)
    list( APPEND ui_list ${CC_PLUGIN_CUSTOM_UI_LIST} )
endif()

target_sources( ${PROJECT_NAME}
    PRIVATE
        ${header_list}
        ${source_list}
        ${moc_list}
        ${ui_list}
        ${qrc_list}
        ${json_list}
)

if( APPLE )
    # put all the plugins we build into one directory
    set( PLUGINS_OUTPUT_DIR "${CMAKE_BINARY_DIR}/ccPlugins" )

    file( MAKE_DIRECTORY "${PLUGINS_OUTPUT_DIR}" )

    set_target_properties( ${PROJECT_NAME}
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PLUGINS_OUTPUT_DIR}"
    )

    install( TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CLOUDCOMPARE_MAC_PLUGIN_DIR} COMPONENT Runtime )
    set( CLOUDCOMPARE_PLUGINS ${CLOUDCOMPARE_PLUGINS} ${CLOUDCOMPARE_MAC_PLUGIN_DIR}/lib${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX} CACHE INTERNAL "CloudCompare plugin list")
  elseif( UNIX )
    install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare/plugins 0 )
  else()
    install_shared( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} 1 /plugins )
endif()

#GL filters and IO plugins also go the the ccViewer 'plugins' sub-folder
if( ${OPTION_BUILD_CCVIEWER} )
    get_target_property( IS_IO_PLUGIN ${PROJECT_NAME} IO_PLUGIN )
    if( CC_SHADER_FOLDER OR IS_IO_PLUGIN )
        if( APPLE )
            install( TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CCVIEWER_MAC_PLUGIN_DIR} COMPONENT Runtime )
            set( CCVIEWER_PLUGINS ${CCVIEWER_PLUGINS} ${CCVIEWER_MAC_PLUGIN_DIR}/lib${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX} CACHE INTERNAL "ccViewer plugin list")
        elseif( UNIX )
          install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare/plugins 0 )
        else()
          install_shared( ${PROJECT_NAME} ${CCVIEWER_DEST_FOLDER} 1 /plugins )
        endif()
    endif()
endif()

#'GL filter' plugins specifics
if( CC_SHADER_FOLDER )
	# copy shader dirs into our shadow build directory
	file( COPY shaders DESTINATION "${CMAKE_BINARY_DIR}" )

	# install the shader files
    file( GLOB shaderFiles shaders/${CC_SHADER_FOLDER}/*.frag shaders/${CC_SHADER_FOLDER}/*.vert )
    foreach( filename ${shaderFiles} )
        if( APPLE )
            install( FILES ${filename} DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/${CC_SHADER_FOLDER} )
            if( ${OPTION_BUILD_CCVIEWER} )
                install( FILES ${filename} DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders/${CC_SHADER_FOLDER} )
            endif()
        elseif ( UNIX )
          install( FILES ${filename} DESTINATION share/cloudcompare/shaders/${CC_SHADER_FOLDER} )
        else()
            install_ext( FILES ${filename} ${CLOUDCOMPARE_DEST_FOLDER} /shaders/${CC_SHADER_FOLDER} )
            if( ${OPTION_BUILD_CCVIEWER} )
                install_ext( FILES ${filename} ${CCVIEWER_DEST_FOLDER} /shaders/${CC_SHADER_FOLDER} )
            endif()
        endif()
    endforeach()
endif()

### END OF DEFAULT CC PLUGIN CMAKE SCRIPT ###
