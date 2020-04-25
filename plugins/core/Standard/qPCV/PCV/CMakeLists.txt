include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( PCV_LIB )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

set_target_properties( ${PROJECT_NAME} PROPERTIES
    AUTOMOC OFF
    AUTORCC OFF
    AUTOUIC OFF
)

target_link_libraries( ${PROJECT_NAME}
    CCCoreLib
    Qt5::Gui
    Qt5::OpenGL
)

if (WIN32)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_USE_AS_DLL )
endif()
