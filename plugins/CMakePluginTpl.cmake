### DEFAULT CC "STANDARD" PLUGIN CMAKE SCRIPT ###

include( CMakePolicies NO_POLICY_SCOPE )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CloudCompare_SOURCE_DIR}/../common )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_IO_LIB_SOURCE_DIR} )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
if( MSVC )
    include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()
include_directories( ${QCC_GL_LIB_SOURCE_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

# force link with interface implementations
list( APPEND source_list ${CloudComparePlugins_SOURCE_DIR}/ccDefaultPluginInterface.cpp )

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


qt5_wrap_ui( generated_ui_list ${ui_list} )
qt5_add_resources( generated_qrc_list ${qrc_list} )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list} ${json_list} )

# Add custom default preprocessor definitions
if (OPTION_GL_QUAD_BUFFER_SUPPORT)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_GL_WINDOW_USE_QWINDOW )
endif()

# Plugins need the QT_NO_DEBUG preprocessor in release!
if( WIN32 )
	target_compile_definitions( ${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:QT_NO_DEBUG> )
endif()

target_link_libraries( ${PROJECT_NAME}
	CC_FBO_LIB
	CC_CORE_LIB
	QCC_DB_LIB
	QCC_IO_LIB
	QCC_GL_LIB )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL Qt5::Concurrent)

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
    if( CC_SHADER_FOLDER OR CC_IS_IO_PLUGIN )
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
