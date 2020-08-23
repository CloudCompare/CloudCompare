# ------------------------------------------------------------------------------
# GDAL support for CloudCompare
# ------------------------------------------------------------------------------

option( OPTION_USE_GDAL "Build with GDAL support" OFF )
if( ${OPTION_USE_GDAL} )
	find_package( GDAL REQUIRED )
	
	if ( WIN32 )
		option( GDAL_VERSION_3 "Check to compile with GDAL v3 instead of GDAL v2" OFF )
	endif()
	
	if ( NOT GDAL_FOUND )
		message( SEND_ERROR "GDAL package not found" )
	else()
		include_directories( ${GDAL_INCLUDE_DIR} )
		if( WIN32 )
			set( GDAL_BIN_DIR ${GDAL_INCLUDE_DIR}/../bin CACHE PATH "GDAL DLLs folder" )
		endif()
	endif()
endif()

# Link project with GDAL library
function( target_link_GDAL ) # 2 arguments: ARGV0 = project name / ARGV1 = destination folder name
	if( ${OPTION_USE_GDAL} )
		if( GDAL_FOUND )
	
	target_link_libraries( ${ARGV0} ${GDAL_LIBRARY} )
	target_compile_definitions( ${ARGV0} PUBLIC CC_GDAL_SUPPORT )
	
	if( WIN32 )
		#install DLLs
		
		message( STATUS "Looked for GDAL DLLs in: " ${GDAL_BIN_DIR} )
		file( GLOB GDAL_DLL_FILES ${GDAL_BIN_DIR}/gdal*.dll )
		#message( STATUS ${GDAL_DLL_FILES} )

		if (ARGV1)
			copy_files("${GDAL_DLL_FILES}" ${ARGV1} ) #mind the quotes!
		endif()
		
		if (GDAL_VERSION_3)
			set (  GDAL_DLL_FILES2	${GDAL_BIN_DIR}/ogdi.dll
									${GDAL_BIN_DIR}/expat.dll
									${GDAL_BIN_DIR}/libpq.dll
									${GDAL_BIN_DIR}/szip.dll
									${GDAL_BIN_DIR}/netcdf.dll
									${GDAL_BIN_DIR}/sqlite3.dll
									${GDAL_BIN_DIR}/spatialite.dll
									${GDAL_BIN_DIR}/libmysql.dll
									${GDAL_BIN_DIR}/geos.dll
									${GDAL_BIN_DIR}/geos_c.dll
									${GDAL_BIN_DIR}/libcurl.dll
									${GDAL_BIN_DIR}/libpng16.dll
									${GDAL_BIN_DIR}/liblzma.dll
									${GDAL_BIN_DIR}/openjp2.dll
									${GDAL_BIN_DIR}/proj_6_3.dll
									${GDAL_BIN_DIR}/zstd.dll
									${GDAL_BIN_DIR}/iconv-2.dll
									${GDAL_BIN_DIR}/iconv.dll
									${GDAL_BIN_DIR}/freexl.dll
									${GDAL_BIN_DIR}/zlib1.dll
									${GDAL_BIN_DIR}/libssl-1_1-x64.dll
									${GDAL_BIN_DIR}/libcrypto-1_1-x64.dll
									${GDAL_BIN_DIR}/hdf5_hl.dll
									${GDAL_BIN_DIR}/hdf5.dll
									${GDAL_BIN_DIR}/libxml2.dll
									${GDAL_BIN_DIR}/lwgeom.dll
									${GDAL_BIN_DIR}/xerces-c_3_2.dll
			)
		else()
			set (  GDAL_DLL_FILES2	${GDAL_BIN_DIR}/expat.dll
									${GDAL_BIN_DIR}/libpq.dll
									${GDAL_BIN_DIR}/szip.dll
									${GDAL_BIN_DIR}/netcdf.dll
									${GDAL_BIN_DIR}/sqlite3.dll
									${GDAL_BIN_DIR}/spatialite.dll
									${GDAL_BIN_DIR}/libmysql.dll
									${GDAL_BIN_DIR}/geos.dll
									${GDAL_BIN_DIR}/geos_c.dll
									${GDAL_BIN_DIR}/libcurl.dll
									${GDAL_BIN_DIR}/openjp2.dll
									${GDAL_BIN_DIR}/proj.dll
									${GDAL_BIN_DIR}/iconv.dll
									${GDAL_BIN_DIR}/freexl.dll
									${GDAL_BIN_DIR}/zlib1.dll
									${GDAL_BIN_DIR}/hdf5_hl.dll
									${GDAL_BIN_DIR}/hdf5.dll
									${GDAL_BIN_DIR}/libxml2.dll
									${GDAL_BIN_DIR}/xerces-c_3_1.dll
									${GDAL_BIN_DIR}/ssleay32.dll
									${GDAL_BIN_DIR}/libeay32.dll
			)
		endif()
		
		message( STATUS ${GDAL_DLL_FILES2} )
		if (ARGV1)
			copy_files("${GDAL_DLL_FILES2}" ${ARGV1} ) #mind the quotes!			
		endif()
	endif()

		else()
			message( SEND_ERROR "GDAL package not found: can't link" )
		endif()	
	endif()	
endfunction()
