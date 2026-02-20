# CC custom CMake script for Ribbonsoft's DXF Lib
# https://www.qcad.org/en/dxflib-downloads

project( dxflib )

add_library( ${PROJECT_NAME} STATIC )

add_subdirectory( src )

if (WIN32)
	target_compile_definitions( ${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE )
endif()

target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_DXF_SUPPORT )
