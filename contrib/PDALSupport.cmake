# pdal
OPTION( OPTION_PDAL_LAS "Build with PDAL to support las format" OFF)
if ( ${OPTION_PDAL_LAS} )
    find_package(PDAL 1.0.0 REQUIRED CONFIG)
    if ( NOT PDAL_FOUND )
        message( SEND_ERROR "PDAL package not found!")
    else()
        include_directories(${PDAL_INCLUDE_DIRS})
        link_directories(${PDAL_LIBRARY_DIRS})
    endif()
endif()


# Link project with PDAL library
function( target_link_PDAL ) # 2 arguments: ARGV0 = project name / ARGV1 = base lib export folder (optional)
    if( ${OPTION_PDAL_LAS} )
        if( PDAL_FOUND )
            add_definitions(${PDAL_DEFINITIONS})
            target_link_libraries(${ARGV0} ${PDAL_LIBRARIES})
            set_property( TARGET ${ARGV0} APPEND PROPERTY COMPILE_DEFINITIONS CC_LAS_SUPPORT )

            #Win32 copy dll?!
            if( WIN32 )
                if ( MSVC_VERSION EQUAL 1910 ) # Visual Studio 2017
                    add_definitions(-DWIN32_LEAN_AND_MEAN)
                endif()
                target_link_libraries(${ARGV0} "pdal_util")
            endif()

        else()
            message( SEND_ERROR "PDAL package not found: can't link" )
        endif()
    endif()
endfunction()

