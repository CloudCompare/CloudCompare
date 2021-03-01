set( CC_PLUGIN_TARGET_LIST "" CACHE INTERNAL "Internal plugin list" )

# AddPlugin should be called once for each plugin.
# This function sets up a target for the plugin, sets up default properties, and sets the target
# to link to the necessary libraries.
#
# Arguments:
#	NAME The name of the plugin (this is the target)
#	TYPE One of "gl", "io", or "standard". If TYPE is not specified, it will default to "standard".
#	SHADER_FOLDER If it is a gl plugin, this is the folder containing the shaders. Ignored otherwise.
function( AddPlugin )
	cmake_parse_arguments(
			ADD_PLUGIN
			""
			"NAME;TYPE;SHADER_FOLDER"
			""
			${ARGN}
	)
	
	# For readability
	set( PLUGIN_TARGET "${ADD_PLUGIN_NAME}" )
	
	set( CC_PLUGIN_TARGET_LIST "${CC_PLUGIN_TARGET_LIST};${PLUGIN_TARGET}" CACHE INTERNAL "Internal plugin list" )
	
	# First check our TYPE
	
	# If none given, make it "standard"
	if ( NOT ADD_PLUGIN_TYPE )
		set( ADD_PLUGIN_TYPE "standard" )
	endif()
	
	# Check that we were given a valid type
	set( VALID_TYPES "gl" "io" "standard" )
	if( NOT "${ADD_PLUGIN_TYPE}" IN_LIST VALID_TYPES )		  
		# In cmake 3.12:
		# list( JOIN VALID_TYPES ", " VALID_TYPES_STR )
		string( REPLACE ";" ", " VALID_TYPES_STR "${VALID_TYPES}" )
		
		message( FATAL_ERROR "AddPlugin: Did not find proper TYPE. Valid values are: ${VALID_TYPES_STR}" )
	endif()
	
	# Create our target
	add_library( ${PLUGIN_TARGET} SHARED )
	
	# Set default properties
	set_target_properties( ${PLUGIN_TARGET}
		PROPERTIES
			AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
			AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui
			PLUGIN_TYPE "${ADD_PLUGIN_TYPE}"
	)
	
	# Keep track of our shader folder if it was given
	if( ("${ADD_PLUGIN_TYPE}" STREQUAL "gl") AND ADD_PLUGIN_SHADER_FOLDER )
		# Because they are added relative to the CMakeLists file, add the path
		set( SHADER_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${ADD_PLUGIN_SHADER_FOLDER}" )
		
		# Check that the folder exists
		if( NOT EXISTS "${SHADER_FOLDER}" )
			message( FATAL_ERROR "AddPlugin: SHADER_FOLDER does not exist: ${SHADER_FOLDER}" )
		endif()
		
		# Add it as a property
		set_target_properties( ${PLUGIN_TARGET}
			PROPERTIES
				SHADER_FOLDER_NAME "${ADD_PLUGIN_SHADER_FOLDER}"
				SHADER_FOLDER_PATH "${SHADER_FOLDER}"
		)
	endif()
	
	if( WIN32 )
		# Plugins need the QT_NO_DEBUG preprocessor in release!
		target_compile_definitions( ${PLUGIN_TARGET} PRIVATE $<$<CONFIG:Release>:QT_NO_DEBUG> )
	endif()
	
	# Add .qrc file to sources
	get_filename_component( FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
	set( QRC_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${FOLDER_NAME}.qrc" )
	if( EXISTS "${QRC_FILE}" )
		target_sources( ${PLUGIN_TARGET} PRIVATE ${QRC_FILE} )
	else()
		message( FATAL_ERROR "Missing qrc file: ${QRC_FILE}" )
	endif()

	# Add info.json to sources
	set( INFO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/info.json" )
	if( EXISTS "${INFO_FILE}" )
		target_sources( ${PLUGIN_TARGET} PRIVATE ${INFO_FILE} )
	else()
		message( FATAL_ERROR "Missing info.json file: ${INFO_FILE}" )
	endif()
   
	# Link to required libraries
	target_link_libraries( ${PLUGIN_TARGET}
		CCCoreLib
		CCPluginAPI
		CCPluginStub
	)

	# On macOS, copy the plugin to the ccPlugins directory at the top level
	# post build so we can find it without installing everything.
	if( APPLE )
		set( PLUGINS_OUTPUT_DIR "${CMAKE_BINARY_DIR}/ccPlugins" )
		
		if( NOT EXISTS PLUGINS_OUTPUT_DIR )
			file( MAKE_DIRECTORY "${PLUGINS_OUTPUT_DIR}" )
		endif()
		
		add_custom_command( TARGET ${PLUGIN_TARGET} POST_BUILD
			COMMAND
				${CMAKE_COMMAND} -E copy_if_different
					$<TARGET_FILE:${PLUGIN_TARGET}>
					${PLUGINS_OUTPUT_DIR}
		)
	endif()

	message( STATUS "Added ${ADD_PLUGIN_TYPE} plugin: ${ADD_PLUGIN_NAME}")
endfunction()

# Documentation of custom properties
define_property( TARGET
	PROPERTY
		PLUGIN_TYPE
	BRIEF_DOCS
		"Type of plugin: gl, io, or standard."
	FULL_DOCS
		"Type of plugin: gl, io, or standard. Used to decide where to install this plugin."
)

define_property( TARGET
	PROPERTY
		SHADER_FOLDER_NAME
	BRIEF_DOCS
		"Name of the shader folder for a gl plugin."
	FULL_DOCS
		"Name of the shader folder for a gl plugin. Used as the name of the folder when installing the shader files."
)

define_property( TARGET
	PROPERTY
		SHADER_FOLDER_PATH
	BRIEF_DOCS
		"Path to the folder containing the shader files for a gl plugin."
	FULL_DOCS
		"Path to the folder containing the shader files for a gl plugin. Files are copied from here when installing."
)
