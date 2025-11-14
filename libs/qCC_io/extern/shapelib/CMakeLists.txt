# CC custom CMake script for shapelib
# https://github.com/OSGeo/shapelib

project( shapelib )

add_library( ${PROJECT_NAME} STATIC )

target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/shptree.c
        ${CMAKE_CURRENT_LIST_DIR}/shpopen.c
        ${CMAKE_CURRENT_LIST_DIR}/shapefil.h
        ${CMAKE_CURRENT_LIST_DIR}/safileio.c
        ${CMAKE_CURRENT_LIST_DIR}/dbfopen.c
)

if ( UNIX )
    set_target_properties( ${PROJECT}
        PROPERTIES
            POSITION_INDEPENDENT_CODE ON
            CXX_VISIBILITY_PRESET hidden
    )
endif()

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
