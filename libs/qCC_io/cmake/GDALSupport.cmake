# ------------------------------------------------------------------------------
# GDAL support for CloudCompare
# ------------------------------------------------------------------------------

option( OPTION_USE_GDAL "Build with GDAL support" OFF )
if( ${OPTION_USE_GDAL} )
	find_package( GDAL REQUIRED )
	
	if ( NOT GDAL_FOUND )
		message( SEND_ERROR "GDAL package not found" )
	else()
		target_include_directories( ${PROJECT_NAME} PUBLIC ${GDAL_INCLUDE_DIR} )
		if( WIN32 )
			set( GDAL_BIN_DIR ${GDAL_INCLUDE_DIR}/../bin CACHE PATH "GDAL DLLs folder" )
		endif()
	endif()
endif()

# Link project with GDAL library
function( target_link_GDAL ) # 2 arguments: ARGV0 = project name / ARGV1 = base lib export folder (optional)
	if( NOT GDAL_FOUND )
		message( FATAL_ERROR "GDAL package not found" )
	endif()
	
	target_link_libraries( ${ARGV0} ${GDAL_LIBRARY} )
	target_compile_definitions( ${ARGV0} PUBLIC CC_GDAL_SUPPORT )
	
	if( WIN32 )
		#install DLLs
		if ( ARGV1 )
			message( STATUS "Looking for GDAL DLLs" )

			file( GLOB GDAL_DLL_FILES ${GDAL_BIN_DIR}/*.dll )
			message( STATUS "Looked in: " ${GDAL_BIN_DIR} )
			message( STATUS ${GDAL_DLL_FILES} )

			file( GLOB GDAL_DLL_FILES2 ${GDAL_INCLUDE_DIR}/../bin/*.dll )
			message( STATUS "Looked in: " ${GDAL_INCLUDE_DIR}/../bin )
			message( STATUS ${GDAL_DLL_FILES2} )

			copy_files("${GDAL_DLL_FILES}" "${ARGV1}" ) #mind the quotes!
			copy_files("${GDAL_DLL_FILES2}" "${ARGV1}" ) #mind the quotes!
		endif()
	endif()
endfunction()
