include(FindPackageHandleStandardArgs)
# TODO laszip_api && laszip version from header

find_library(LASZIP_LIBRARY
        NAMES laszip3 laszip
        HINTS /usr/lib
        /usr/local/lib
        )

find_path(LASZIP_INCLUDE_DIR
        NAMES laszip
        HINTS /usr/include
        /usr/local/include
        )

if (WIN32)
    find_file(LASZIP_DLL
            NAMES laszip3.dll
            HINTS "${LASZIP_INCLUDE_DIR}/../bin"
            )
else ()
    set(LASZIP_DLL "Dummy Value so that handle args does not fail")
endif ()

message(DEBUG "LASZIP_LIBRARY: ${LASZIP_LIBRARY}")
message(DEBUG "LASZIP_DLL: ${LASZIP_DLL}")

find_package_handle_standard_args(LASzip
        REQUIRED_VARS LASZIP_LIBRARY LASZIP_INCLUDE_DIR LASZIP_DLL
        HANDLE_COMPONENTS
        )

if (LASzip_FOUND)
    mark_as_advanced(LASZIP_LIBRARY LASZIP_INCLUDE_DIR)
endif ()

if (LASzip_FOUND AND NOT TARGET LASzip::LASzip)
    add_library(LASzip::LASzip SHARED IMPORTED)
    if (WIN32)
        set_target_properties(LASzip::LASzip PROPERTIES
                IMPORTED_LOCATION ${LASZIP_DLL}
                IMPORTED_IMPLIB ${LASZIP_LIBRARY}
                )
    else ()
        set_target_properties(LASzip::LASzip PROPERTIES
                IMPORTED_LOCATION ${LASZIP_LIBRARY})
    endif ()
    target_include_directories(LASzip::LASzip INTERFACE ${LASZIP_INCLUDE_DIR})
endif ()
