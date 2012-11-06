### DEFAULT CC "STANDARD" PLUGIN CMAKE SCRIPT ###

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${GLEW_SOURCE_DIR}/include )
include_directories( ${CC_FBO_SOURCE_DIR}/include )
include_directories( ${CC_DLL_SOURCE_DIR}/include )
include_directories( ${QCC_DB_DLL_SOURCE_DIR} )
if( MSVC )
include_directories( ${QCC_DB_DLL_SOURCE_DIR}/msvc )
endif()
include_directories( ${qCC_SOURCE_DIR} )
include_directories( ${qCC_SOURCE_DIR}/db_tree )
include_directories( ${qCC_BINARY_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)
file( GLOB ui_list *.ui )
file( GLOB qrc_list *.qrc )
file( GLOB rc_list *.rc )

# find Qt mocable files
find_mocable_files( mocable_list ${header_list} )
qt4_wrap_cpp( moc_list ${mocable_list} )
QT4_WRAP_UI( generated_ui_list ${ui_list} )
QT4_ADD_RESOURCES( generated_qrc_list ${qrc_list} )

if ( CC_PLUGIN_CUSTOM_HEADER_LIST )
	list( APPEND header_list ${CC_PLUGIN_CUSTOM_HEADER_LIST} )
endif()

if ( CC_PLUGIN_CUSTOM_SOURCE_LIST )
	list( APPEND source_list ${CC_PLUGIN_CUSTOM_SOURCE_LIST} )
endif()

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} ${moc_list} ${generated_ui_list} ${generated_qrc_list})

# by default, plugins shared libraries should be output to the qCC's 'plugins' subdirectory (for easier debug)
# DGM: how to do this properly? (we also miss qCC_db and CCLib, as well as shaders, Qt dlls, etc.)
#if ( CMAKE_CONFIGURATION_TYPES )
#	set_target_properties( ${PROJECT_NAME} PROPERTIES ${SHARED_LIB_TYPE}_OUTPUT_DIRECTORY_RELEASE ${qCC_BINARY_DIR}/Release/plugins )
#	set_target_properties( ${PROJECT_NAME} PROPERTIES ${SHARED_LIB_TYPE}_OUTPUT_DIRECTORY_DEBUG ${qCC_BINARY_DIR}/Debug/plugins )
#else()
#	set_target_properties( ${PROJECT_NAME} PROPERTIES ${SHARED_LIB_TYPE}_OUTPUT_DIRECTORY ${qCC_BINARY_DIR}/plugins )
#endif()

# All plugins depend (by default) on qCC
add_dependencies( ${PROJECT_NAME} qCC )

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom default prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )
if( WIN32 )
    set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

# Plugins need the QT_NO_DEBUG preprocessor in release!
if( NOT CMAKE_CONFIGURATION_TYPES )
    set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS QT_NO_DEBUG )
else()
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE QT_NO_DEBUG)
endif()

target_link_libraries( ${PROJECT_NAME} GLEW )
target_link_libraries( ${PROJECT_NAME} CC_FBO )
target_link_libraries( ${PROJECT_NAME} CC_DLL )
target_link_libraries( ${PROJECT_NAME} QCC_DB_DLL )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

install_shared( ${PROJECT_NAME} ${qCC_dest_release}/plugins ${qCC_dest_debug}/plugins )

if( CC_SHADER_FOLDER )
	file( GLOB shaderFiles shaders/${CC_SHADER_FOLDER}/*.frag shaders/${CC_SHADER_FOLDER}/*.vert )
	foreach( filename ${shaderFiles} )
		install_ext( FILES ${filename} ${qCC_dest_release}/shaders/${CC_SHADER_FOLDER} ${qCC_dest_debug}/shaders/${CC_SHADER_FOLDER} )
	endforeach()
endif()

### END OF DEFAULT CC PLUGIN CMAKE SCRIPT ###
