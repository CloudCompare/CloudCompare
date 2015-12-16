cmake_minimum_required(VERSION 2.8)


include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories( ${GLEW_LIB_SOURCE_DIR}/include )

project( CC_FBO_LIB )

file( GLOB header_list include/*.h )
file( GLOB source_list src/*.cpp )



add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

target_link_libraries( ${PROJECT_NAME} GLEW_LIB )


if ( USE_QT5 )
	qt5_use_modules(${PROJECT_NAME} Core ) # Widgets
else()
    target_link_libraries( ${PROJECT_NAME} ${QT_LIBRARIES} )
endif()

# Add custom prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS USE_GLEW GLEW_STATIC )

