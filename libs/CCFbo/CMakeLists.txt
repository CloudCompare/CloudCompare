cmake_minimum_required(VERSION 2.8)

include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories( ${GLEW_LIB_SOURCE_DIR}/include )

project( CC_FBO_LIB )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )

if ( NOT USE_QT5 )
	# find Qt mocable files
	find_mocable_files( mocable_list ${header_list} )
	qt4_wrap_cpp( moc_list ${mocable_list} )
endif()

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} GLEW_LIB )
target_link_libraries( ${PROJECT_NAME} ${EXTERNAL_LIBS_LIBRARIES} )

if ( USE_QT5 )
	qt5_use_modules(${PROJECT_NAME} Core ) # Widgets
endif()

# Default preprocessors
set_default_cc_preproc( ${PROJECT_NAME} )

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )

