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
function( target_link_GDAL ) # ARGV0 = project name
	if( NOT GDAL_FOUND )
		message( FATAL_ERROR "GDAL package not found" )
	endif()
	
	target_link_libraries( ${ARGV0} ${GDAL_LIBRARY} )
	target_compile_definitions( ${ARGV0} PUBLIC CC_GDAL_SUPPORT )
	
	if( WIN32 )
		#install DLLs
		message( STATUS "Looking for GDAL DLLs in: " ${GDAL_BIN_DIR} )
		file( GLOB GDAL_DLL_FILES ${GDAL_BIN_DIR}/gdal*.dll )
				#message( ${GDAL_DLL_FILES} )
                message( STATUS "GDAL VERSION: " ${GDAL_VERSION} )
                if (GDAL_VERSION EQUAL 3.2)
                    set ( GDAL_DEP_DLL_FILES ${GDAL_BIN_DIR}/xerces-c_3_2.dll
                        ${GDAL_BIN_DIR}/libexpat.dll
                        ${GDAL_BIN_DIR}/LIBPQ.dll
                        ${GDAL_BIN_DIR}/hdf.dll
                        ${GDAL_BIN_DIR}/mfhdf.dll
                        ${GDAL_BIN_DIR}/cfitsio.dll
                        ${GDAL_BIN_DIR}/libjpeg.dll
                        ${GDAL_BIN_DIR}/netcdf.dll
                        ${GDAL_BIN_DIR}/geotiff.dll
                        ${GDAL_BIN_DIR}/tiff.dll
                        ${GDAL_BIN_DIR}/proj_8_0.dll
                        ${GDAL_BIN_DIR}/sqlite3.dll
                        ${GDAL_BIN_DIR}/spatialite.dll
                        ${GDAL_BIN_DIR}/geos_c.dll
                        ${GDAL_BIN_DIR}/hdf5.dll
                        ${GDAL_BIN_DIR}/hdf5_cpp.dll
                        ${GDAL_BIN_DIR}/libkea.dll
                        ${GDAL_BIN_DIR}/libcurl.dll
                        ${GDAL_BIN_DIR}/libpng16.dll
                        ${GDAL_BIN_DIR}/openjp2.dll
                        ${GDAL_BIN_DIR}/poppler.dll
                        ${GDAL_BIN_DIR}/iconv.dll
                        ${GDAL_BIN_DIR}/libwebp.dll
                        ${GDAL_BIN_DIR}/tiledb.dll
                        ${GDAL_BIN_DIR}/freexl.dll
                        ${GDAL_BIN_DIR}/libssl-1_1-x64.dll
                        ${GDAL_BIN_DIR}/libcrypto-1_1-x64.dll
                        ${GDAL_BIN_DIR}/gssapi64.dll
                        ${GDAL_BIN_DIR}/xdr.dll
                        ${GDAL_BIN_DIR}/hdf5_hl.dll
                        ${GDAL_BIN_DIR}/zip.dll
                        ${GDAL_BIN_DIR}/zlib.dll
                        ${GDAL_BIN_DIR}/liblzma.dll
                        ${GDAL_BIN_DIR}/zstd.dll
                        ${GDAL_BIN_DIR}/libxml2.dll
                        ${GDAL_BIN_DIR}/charset.dll
                        ${GDAL_BIN_DIR}/geos.dll
                        ${GDAL_BIN_DIR}/libssh2.dll
                        ${GDAL_BIN_DIR}/freetype.dll
                        ${GDAL_BIN_DIR}/aws-cpp-sdk-s3.dll
                        ${GDAL_BIN_DIR}/aws-cpp-sdk-core.dll
                        ${GDAL_BIN_DIR}/aws-cpp-sdk-identity-management.dll
                        ${GDAL_BIN_DIR}/libbz2.dll
                        ${GDAL_BIN_DIR}/liblz4.dll
                        ${GDAL_BIN_DIR}/krb5_64.dll
                        ${GDAL_BIN_DIR}/comerr64.dll
                        ${GDAL_BIN_DIR}/k5sprt64.dll
                        ${GDAL_BIN_DIR}/aws-c-event-stream.dll
                        ${GDAL_BIN_DIR}/aws-c-common.dll
                        ${GDAL_BIN_DIR}/aws-cpp-sdk-cognito-identity.dll
                        ${GDAL_BIN_DIR}/aws-cpp-sdk-sts.dll
                        ${GDAL_BIN_DIR}/aws-checksums.dll
                        # for pdal
                        ${GDAL_BIN_DIR}/laszip3.dll
                        )
                elseif (GDAL_VERSION EQUAL 3.3)
                    set ( GDAL_DEP_DLL_FILES ${GDAL_BIN_DIR}/xerces-c_3_2.dll
                        ${GDAL_BIN_DIR}/cfitsio.dll
                        ${GDAL_BIN_DIR}/freexl.dll
                        ${GDAL_BIN_DIR}/geos.dll
                        ${GDAL_BIN_DIR}/geos_c.dll
                        ${GDAL_BIN_DIR}/iconv-2.dll
                        ${GDAL_BIN_DIR}/hdf5.dll
                        ${GDAL_BIN_DIR}/hdf5_cpp.dll
                        ${GDAL_BIN_DIR}/hdf5_hl.dll
                        ${GDAL_BIN_DIR}/hdf.dll
                        ${GDAL_BIN_DIR}/libcrypto-1_1-x64.dll
                        ${GDAL_BIN_DIR}/libcurl.dll
                        ${GDAL_BIN_DIR}/libexpat.dll
                        ${GDAL_BIN_DIR}/libkea.dll
						${GDAL_BIN_DIR}/libmysql.dll
                        ${GDAL_BIN_DIR}/libpq.dll
                        ${GDAL_BIN_DIR}/libpng16.dll
                        ${GDAL_BIN_DIR}/libssl-1_1-x64.dll
                        ${GDAL_BIN_DIR}/libxml2.dll
                        ${GDAL_BIN_DIR}/mfhdf.dll
                        ${GDAL_BIN_DIR}/netcdf.dll
                        ${GDAL_BIN_DIR}/openjp2.dll
                        ${GDAL_BIN_DIR}/ogdi.dll
                        ${GDAL_BIN_DIR}/proj_7_2.dll
                        ${GDAL_BIN_DIR}/spatialite.dll
                        ${GDAL_BIN_DIR}/sqlite3.dll
                        ${GDAL_BIN_DIR}/szip.dll
                        ${GDAL_BIN_DIR}/tiff.dll
                        ${GDAL_BIN_DIR}/xdr.dll
                        ${GDAL_BIN_DIR}/zlib.dll
                        ${GDAL_BIN_DIR}/zstd.dll
                        # for pdal
                        #${GDAL_BIN_DIR}/laszip3.dll
                        )
                elseif (GDAL_VERSION EQUAL 3.4)
                    set ( GDAL_DEP_DLL_FILES ${GDAL_BIN_DIR}/xerces-c_3_2.dll
                        ${GDAL_BIN_DIR}/libexpat.dll
                        ${GDAL_BIN_DIR}/LIBPQ.dll
                        ${GDAL_BIN_DIR}/hdf.dll
                        ${GDAL_BIN_DIR}/mfhdf.dll
                        ${GDAL_BIN_DIR}/cfitsio.dll
                        ${GDAL_BIN_DIR}/libjpeg.dll
                        ${GDAL_BIN_DIR}/netcdf.dll
                        ${GDAL_BIN_DIR}/geotiff.dll
                        ${GDAL_BIN_DIR}/tiff.dll
                        ${GDAL_BIN_DIR}/proj_9_0.dll
                        ${GDAL_BIN_DIR}/sqlite3.dll
                        ${GDAL_BIN_DIR}/spatialite.dll
                        ${GDAL_BIN_DIR}/geos_c.dll
                        ${GDAL_BIN_DIR}/hdf5.dll
                        ${GDAL_BIN_DIR}/hdf5_cpp.dll
                        ${GDAL_BIN_DIR}/libkea.dll
                        ${GDAL_BIN_DIR}/libcurl.dll
                        ${GDAL_BIN_DIR}/libpng16.dll
                        ${GDAL_BIN_DIR}/openjp2.dll
                        ${GDAL_BIN_DIR}/iconv.dll
                        ${GDAL_BIN_DIR}/tiledb.dll
                        ${GDAL_BIN_DIR}/freexl.dll
                        ${GDAL_BIN_DIR}/libssl-1_1-x64.dll
                        ${GDAL_BIN_DIR}/libcrypto-1_1-x64.dll
                        ${GDAL_BIN_DIR}/gssapi64.dll
                        ${GDAL_BIN_DIR}/xdr.dll
                        ${GDAL_BIN_DIR}/hdf5_hl.dll
                        ${GDAL_BIN_DIR}/zip.dll
                        ${GDAL_BIN_DIR}/zlib.dll
                        ${GDAL_BIN_DIR}/liblzma.dll
                        ${GDAL_BIN_DIR}/zstd.dll
                        ${GDAL_BIN_DIR}/libxml2.dll
                        ${GDAL_BIN_DIR}/charset.dll
                        ${GDAL_BIN_DIR}/geos.dll
                        ${GDAL_BIN_DIR}/libssh2.dll
                        ${GDAL_BIN_DIR}/freetype.dll
                        ${GDAL_BIN_DIR}/libbz2.dll
                        ${GDAL_BIN_DIR}/liblz4.dll
                        ${GDAL_BIN_DIR}/krb5_64.dll
                        ${GDAL_BIN_DIR}/comerr64.dll
                        ${GDAL_BIN_DIR}/k5sprt64.dll
                        ${GDAL_BIN_DIR}/libdeflate.dll
                        ${GDAL_BIN_DIR}/jbig.dll
                        ${GDAL_BIN_DIR}/Lerc.dll
                        ${GDAL_BIN_DIR}/poppler.dll
                        ${GDAL_BIN_DIR}/libwebp.dll
                        ${GDAL_BIN_DIR}/lcms2.dll
                        # for pdal
                        #${GDAL_BIN_DIR}/laszip3.dll
                        )
				elseif (GDAL_VERSION_3)
					set (  GDAL_DEP_DLL_FILES	${GDAL_BIN_DIR}/ogdi.dll
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
					set (  GDAL_DEP_DLL_FILES	${GDAL_BIN_DIR}/expat.dll
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
		
		#message( STATUS ${GDAL_DLL_FILES} )
		#message( STATUS ${GDAL_DEP_DLL_FILES} )
		copy_files("${GDAL_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" 1 ) #mind the quotes!
		copy_files("${GDAL_DEP_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" 1 ) #mind the quotes!
		
		if (${OPTION_BUILD_CCVIEWER})
			copy_files("${GDAL_DLL_FILES}" "${CCVIEWER_DEST_FOLDER}" 1 ) #mind the quotes!
			copy_files("${GDAL_DEP_DLL_FILES}" "${CCVIEWER_DEST_FOLDER}" 1 ) #mind the quotes!
		endif()

	endif()
endfunction()
