### DEFAULT CC "STANDARD" PLUGIN CMAKE SCRIPT ###

include( CMakePolicies NO_POLICY_SCOPE )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

if ( CC_PLUGIN_CUSTOM_HEADER_LIST )
    list( APPEND header_list ${CC_PLUGIN_CUSTOM_HEADER_LIST} )
endif()

if ( CC_PLUGIN_CUSTOM_SOURCE_LIST )
    list( APPEND source_list ${CC_PLUGIN_CUSTOM_SOURCE_LIST} )
endif()

target_sources( ${PROJECT_NAME}
    PRIVATE
        ${header_list}
        ${source_list}
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

get_target_property( PLUGIN_TYPE ${PROJECT_NAME} PLUGIN_TYPE )

#GL filters and IO plugins also go the the ccViewer 'plugins' sub-folder
if( ${OPTION_BUILD_CCVIEWER} )
    if( ("${PLUGIN_TYPE}" STREQUAL "gl") OR ("${PLUGIN_TYPE}" STREQUAL "io") )
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
if( "${PLUGIN_TYPE}" STREQUAL "gl" )
    get_target_property( SHADER_FOLDER_NAME ${PROJECT_NAME} SHADER_FOLDER_NAME )
    get_target_property( SHADER_FOLDER_PATH ${PROJECT_NAME} SHADER_FOLDER_PATH )

    if( EXISTS "${SHADER_FOLDER_PATH}" )
        # copy shader dirs into our shadow build directory
        file( COPY shaders DESTINATION "${CMAKE_BINARY_DIR}" )
    
        # install the shader files
        file( GLOB shaderFiles shaders/${SHADER_FOLDER_PATH}/*.frag shaders/${SHADER_FOLDER_PATH}/*.vert )
        foreach( filename ${shaderFiles} )
            if( APPLE )
                install( FILES ${filename} DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/${SHADER_FOLDER_NAME} )
                if( ${OPTION_BUILD_CCVIEWER} )
                    install( FILES ${filename} DESTINATION ${CCVIEWER_MAC_BASE_DIR}/Contents/Shaders/${SHADER_FOLDER_NAME} )
                endif()
            elseif ( UNIX )
              install( FILES ${filename} DESTINATION share/cloudcompare/shaders/${SHADER_FOLDER_NAME} )
            else()
                install_ext( FILES ${filename} ${CLOUDCOMPARE_DEST_FOLDER} /shaders/${SHADER_FOLDER_NAME} )
                if( ${OPTION_BUILD_CCVIEWER} )
                    install_ext( FILES ${filename} ${CCVIEWER_DEST_FOLDER} /shaders/${SHADER_FOLDER_NAME} )
                endif()
            endif()
        endforeach()
    endif()
endif()

unset( PLUGIN_TYPE )
### END OF DEFAULT CC PLUGIN CMAKE SCRIPT ###
