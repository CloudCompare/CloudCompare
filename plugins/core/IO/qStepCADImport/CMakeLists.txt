cmake_minimum_required( VERSION 3.5 )

option( PLUGIN_IO_QSTEP "Check to install the STEP reading file plugin" OFF )

if ( PLUGIN_IO_QSTEP )

	if( UNIX )
		set( OPENCASCADE_INC_DIR "" CACHE PATH "OpenCascade include dir" )
		set( OPENCASCADE_LIB_DIR "" CACHE PATH "OpenCascade library dir" )
	elseif( WIN32 )
		set( OPENCASCADE_DLL_DIR "" CACHE PATH "OpenCascade dll dir" )
		set( OPENCASCADE_TBB_DLL_DIR "" CACHE PATH "TBB dll dir" )
	endif()
	
	if( NOT OPENCASCADE_INC_DIR )
		message( FATAL_ERROR "OpenCascade include dir not specified (OPENCASCADE_INC_DIR)" )
	endif()
	if( NOT OPENCASCADE_LIB_DIR )
		message( FATAL_ERROR "OpenCascade library dir not specified (OPENCASCADE_LIB_DIR)" )
	endif()
	
	project( QSTEP_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )

	target_include_directories( ${PROJECT_NAME} PRIVATE ${OPENCASCADE_INC_DIR} )
	
	set( OC_LIBRARIES TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209 TKShHealing TKTopAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel TKXSBase TKGeomAlgo TKMesh )
	
    	target_link_directories( ${PROJECT_NAME} PRIVATE ${OPENCASCADE_LIB_DIR} )
		target_link_libraries( ${PROJECT_NAME} ${OC_LIBRARIES} )

	if( WIN32 )
		if( NOT OPENCASCADE_DLL_DIR )
			message( FATAL_ERROR "OpenCascade DLL dir not specified (OPENCASCADE_DLL_DIR)" )
		endif()
		if( NOT OPENCASCADE_TBB_DLL_DIR )
			message( FATAL_ERROR "TBB DLL dir not specified (OPENCASCADE_TBB_DLL_DIR)" )
		endif()
		# if ( MSVC_VERSION GREATER_EQUAL 1900 ) # Visual Studio 2017
			# target_compile_definitions( ${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN )
		# endif()
		
		set( OC_DLLS )

        foreach( oc_lib ${OC_LIBRARIES} )
			list( APPEND OC_DLLS "${OPENCASCADE_DLL_DIR}/${oc_lib}.dll" )
		endforeach()
		list( APPEND OC_DLLS "${OPENCASCADE_TBB_DLL_DIR}/tbb.dll" )
		list( APPEND OC_DLLS "${OPENCASCADE_TBB_DLL_DIR}/tbbmalloc.dll" )
		
		#message( ${OC_DLLS} )

		copy_files( "${OC_DLLS}" "${CLOUDCOMPARE_DEST_FOLDER}" 1 )
		
		if( ${OPTION_BUILD_CCVIEWER} )
			copy_files( "${OC_DLLS}" "${CCVIEWER_DEST_FOLDER}" 1 )
		endif()
		
		unset( OC_DLLS )
	endif()


endif()

