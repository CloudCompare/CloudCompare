include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( PCV_LIB )

# no Qt stuff here
set( CMAKE_AUTOMOC OFF )
set( CMAKE_AUTOUIC OFF )
set( CMAKE_AUTORCC OFF )

file( GLOB header_list *.h )
file( GLOB source_list *.cpp )

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} CCCoreLib )

# Add preprocessor definitions
target_compile_definitions( ${PROJECT_NAME} PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS )

if (WIN32)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_USE_AS_DLL )
endif()
