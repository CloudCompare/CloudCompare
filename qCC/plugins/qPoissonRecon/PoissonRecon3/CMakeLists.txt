cmake_minimum_required(VERSION 2.8)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

project( PoissonRecon )

#to remove very annoying "NO ARGUMENTS THAT DEPEND ON A TEMPLATE PARAMETER" errors with G++!
if( UNIX OR MINGW )
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")
endif()

set( SAMPLE_PROJECT_VERSION_MAJOR 3 )

file( GLOB header_list Src/*.h Src_CC_wrap/*.h)
file( GLOB inline_list Src/*.inl)
file( GLOB source_list Src/*.cpp Src_CC_wrap/*.cpp)

add_library( ${PROJECT_NAME} STATIC ${header_list} ${inline_list} ${source_list} )

# Add prepocessor definitions
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS )
