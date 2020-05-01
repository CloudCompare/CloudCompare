# AddPlugin should be called once for each plugin.
# This function sets up a target for the plugin, sets up default properties, and sets the target
# to link to the necessary libraries.
#
# Arguments:
#   NAME The name of the plugin (this is the target)
#   TYPE One of "gl", "io", or "standard". If TYPE is not specified, it will default to "standard".
function( AddPlugin )
    cmake_parse_arguments(
            ADD_PLUGIN
            ""
            "NAME;TYPE"
            ""
            ${ARGN}
        )
    
    # For readability
    set( PLUGIN_TARGET "${ADD_PLUGIN_NAME}" )
    
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
    set_target_properties( ${PLUGIN_TARGET} PROPERTIES
        AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
        AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui
    )
    
    if( "${ADD_PLUGIN_TYPE}" STREQUAL "io" )
        set_target_properties( ${PLUGIN_TARGET}
            PROPERTIES
            IO_PLUGIN 1
        )        
    endif()
    
    if( WIN32 )
        # Plugins need the QT_NO_DEBUG preprocessor in release!
        target_compile_definitions( ${PLUGIN_TARGET} PRIVATE $<$<CONFIG:Release>:QT_NO_DEBUG> )
    endif()
     
    # Link to required libraries
    target_link_libraries( ${PLUGIN_TARGET}
        CCPluginAPI
        CCPluginStub
        QCC_GL_LIB
        Qt5::Concurrent
        Qt5::OpenGL
        Qt5::Widgets
    )

    message( STATUS "Added ${ADD_PLUGIN_TYPE} plugin: ${ADD_PLUGIN_NAME}")
endfunction()

# define_property is for documentation purposes
define_property( TARGET
    PROPERTY
        IS_IO_PLUGIN
    BRIEF_DOCS
        "Indicates that a plugin is an IO plugin."
    FULL_DOCS
        "Indicate that a plugin is an IO plugin. Used to decide where to install this plugin."
)
