# ------------------------------------------------------------------------------
# GDAL support for CloudCompare
# ------------------------------------------------------------------------------

option( OPTION_USE_GDAL "Build with GDAL support" OFF )
if( ${OPTION_USE_GDAL} )
    find_package( GDAL REQUIRED )

    if ( NOT GDAL_FOUND )
        message( SEND_ERROR "GDAL package not found" )
    else()
        target_include_directories( ${PROJECT_NAME} PUBLIC GDAL::GDAL )
    endif()
endif()

# Link project with GDAL library
function( target_link_GDAL ) # ARGV0 = project name
    if( NOT GDAL_FOUND )
        message( FATAL_ERROR "GDAL package not found" )
    endif()

    target_link_libraries( ${ARGV0} PUBLIC GDAL::GDAL )
    target_compile_definitions( ${ARGV0} PUBLIC CC_GDAL_SUPPORT )

    if( WIN32 )
        # Install GDAL DLLs
        install(FILES
            $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
            DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}
            COMPONENT Runtime
        )

        if(OPTION_BUILD_CCVIEWER)
            install(FILES
                $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}>
                DESTINATION ${CCVIEWER_DEST_FOLDER}
                COMPONENT Runtime
            )
            endif()
    endif()

endfunction()
