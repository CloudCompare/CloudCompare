# CC custom CMake script for shapelib
# https://github.com/OSGeo/shapelib

project( shapelib )

add_library( ${PROJECT_NAME} STATIC )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOMOC OFF
    AUTORCC OFF
    AUTOUIC OFF
)

target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/shptree.c
        ${CMAKE_CURRENT_LIST_DIR}/shpopen.c
        ${CMAKE_CURRENT_LIST_DIR}/shapefil.h
        ${CMAKE_CURRENT_LIST_DIR}/safileio.c
        ${CMAKE_CURRENT_LIST_DIR}/dbfopen.c
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_SHP_SUPPORT )
