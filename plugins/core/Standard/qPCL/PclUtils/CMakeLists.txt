cmake_minimum_required (VERSION 3.0)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.12.0")
	# https://cmake.org/cmake/help/git-stage/policy/CMP0074.html
	cmake_policy(SET CMP0074 OLD)
endif()

#requires PCL
find_package(PCL REQUIRED)

if( NOT MSVC )
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread" )
endif()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${EXTERNAL_LIBS_INCLUDE_DIR} )
include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${QCC_DB_LIB_SOURCE_DIR} )
include_directories( ${QCC_GL_LIB_SOURCE_DIR} )
if( MSVC )
  include_directories( ${QCC_DB_LIB_SOURCE_DIR}/msvc )
endif()

project( QPCL_PLUGIN_UTILS_LIB )

include ( CMakePolicies NO_POLICY_SCOPE )

include_directories( ${PCL_INCLUDE_DIRS} )

file( GLOB_RECURSE header_list *.h )
file( GLOB_RECURSE source_list *.cpp )
file( GLOB_RECURSE impl_list *.hpp)
file( GLOB_RECURSE ui_list *.ui )

qt5_wrap_ui(generated_ui_list ${ui_list})

add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} ${impl_list} ${generated_ui_list} )

# Add custom preprocessor definitions
if (WIN32)
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL QCC_DB_USE_AS_DLL )
endif()

#define the PCL_VER_1_6_OR_OLDER preprocessor to compile qPCL with older versions of PCL
if ( PCL_VERSION VERSION_LESS  1.7 ) # VERSION_GREATER Works just like "greater or equal"
        set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS PCL_VER_1_6_OR_OLDER )
endif()

# Luca's PCL patch support
if( PCL_VERSION VERSION_GREATER  1.7 ) #from 1.7 the patch was merged
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS LP_PCL_PATCH_ENABLED )
endif()

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES})

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)

