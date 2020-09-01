if( WIN32 )
	option( OPTION_SUPPORT_3DCONNEXION_DEVICES "Build with 3dConnexion libraries (3D mouse support)" OFF )
	
	if( OPTION_SUPPORT_3DCONNEXION_DEVICES )
		set( 3DXWARE_INCLUDE_DIR "" CACHE PATH "3DxWare include directory" )
		set( 3DXWARE_LIB_DIR "" CACHE PATH "3DxWare libraries directory" )
		
		if( NOT 3DXWARE_INCLUDE_DIR )
			message( SEND_ERROR "3DXWARE include path is not specified (3DXWARE_INCLUDE_DIR)" )
		endif()
		
		if( NOT 3DXWARE_LIB_DIR )
			message( SEND_ERROR "3DXWARE include path is not specified (3DXWARE_LIB_DIR)" )
		endif()
	
		target_sources( ${PROJECT_NAME}
			PRIVATE
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.h
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.cpp
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.h
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.cpp
		)
	
		target_include_directories( ${PROJECT_NAME}
			PUBLIC
				${CMAKE_CURRENT_LIST_DIR}
				${3DXWARE_INCLUDE_DIR}
		)
	
		target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_3DXWARE_SUPPORT )
	endif()
	
	function( target_link_3DXWARE ) # 1 argument: ARGV0 = target
		if( OPTION_SUPPORT_3DCONNEXION_DEVICES )		
			target_link_libraries( ${ARGV0} optimized ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmath.lib)
		
			if ( CMAKE_CONFIGURATION_TYPES )
				target_link_libraries( ${ARGV0} debug ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmathD.lib )
			endif()
		endif()
	endfunction()
endif()
