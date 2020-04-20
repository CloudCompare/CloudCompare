
if ( CC-CORE-LIB_SHARED )
    # Before CMake 3.13, install(TARGETS) would only accept targets created in the same directory scope
    # This makes it difficult to work with submodules.
    # This can be cleaned up when we move to a minimum CMake of 3.13 or higher
    # https://gitlab.kitware.com/cmake/cmake/-/merge_requests/2152
    if ( ${CMAKE_VERSION} VERSION_LESS "3.13.0" )
        # Basic hack: construct the name of the CCCoreLib library ("CCCoreLib_LIBRARY") and install using
        # install(FILES) instead of install(TARGETS)
        
        if ( APPLE OR UNIX )
            set( PREFIX "lib" )
        endif()
        
        if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
            get_target_property( POSTFIX CCCoreLib DEBUG_POSTFIX)
            message( "${POSTFIX}" )
        endif()
        
        set( CCCoreLib_LIBRARY "${CCCoreLib_BINARY_DIR}/${PREFIX}CCCoreLib${POSTFIX}${CMAKE_SHARED_LIBRARY_SUFFIX}" )
                
        if ( WIN32 OR APPLE )
            foreach( dest ${INSTALL_DESTINATIONS} )
                copy_files( "${CCCoreLib_LIBRARY}" "${dest}" 1 )
            endforeach()
        else()
            copy_files( "${CCCoreLib_LIBRARY}" "${CMAKE_INSTALL_LIBDIR}/cloudcompare" )
        endif()
    else()
        if ( WIN32 OR APPLE )
            foreach( dest ${INSTALL_DESTINATIONS} )
                install_shared( CCCoreLib "${dest}" 1 )
            endforeach()
        else()
            install_shared( CCCoreLib "${CMAKE_INSTALL_LIBDIR}/cloudcompare" )
        endif()
    endif()
endif()
