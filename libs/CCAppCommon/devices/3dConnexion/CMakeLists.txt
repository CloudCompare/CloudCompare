if( WIN32_LEGACY_DRIVER ) # former 3DxWare SDK (Windows only)
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
			target_link_libraries( ${ARGV0} PUBLIC optimized ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmath.lib)

			if ( CMAKE_CONFIGURATION_TYPES )
				target_link_libraries( ${ARGV0} PUBLIC debug ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmathD.lib )
			endif()
		endif()
	endfunction()
else() # hidapi-based 3DConnexion support, originally for macOS but also works on Linux and Windows (no 3DxWare SDK required)
	# On macOS we drive the 3DConnexion device directly via hidapi (no 3DxWare SDK).
	# The hidapi sources are pulled in as a Git submodule under extern/hidapi.
	if( NOT TARGET hidapi::hidapi )
		set( HIDAPI_WITH_TESTS OFF CACHE BOOL "" FORCE )
		# Build the hidapi submodule. The path to the source and binary directories is relative to the current CMakeLists.txt file
		# (useful when CloudCompare is a git submodule in another project).
		set( HIDAPI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../extern/hidapi )
		set( HIDAPI_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../../../extern/hidapi )
		add_subdirectory( ${HIDAPI_SOURCE_DIR} ${HIDAPI_BINARY_DIR} EXCLUDE_FROM_ALL )
	endif()

	# Allow target_link_libraries() to act on targets defined in the parent directory.
	cmake_policy( SET CMP0079 NEW )

	option( OPTION_SUPPORT_3DCONNEXION_DEVICES "Build with hidapi (3D mouse support on macOS)" ON )
	# Optional verbose logging of raw HID reports to the CloudCompare console.
	# Off by default; turn on to diagnose device-specific report formats.
	option( OPTION_HID_DEBUG "Enable verbose HID report logging (3D mouse)" OFF )

	if( OPTION_SUPPORT_3DCONNEXION_DEVICES )
		target_sources( ${PROJECT_NAME}
			PRIVATE
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.h
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.cpp
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.h
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.cpp
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput_mac.h
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput_mac.cpp
		)

		target_include_directories( ${PROJECT_NAME}
			PUBLIC
				${CMAKE_CURRENT_LIST_DIR}
			# The hidapi header (hidapi.h) lives in extern/hidapi/hidapi/.
			# Add the path explicitly so we don't depend solely on transitive
			# propagation from the hidapi::hidapi target.
			PRIVATE
				${HIDAPI_SOURCE_DIR}/hidapi
		)

		target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_3DXWARE_SUPPORT )
		# Used by Mouse3DInput.{h,cpp} and Mouse3DInput_mac.{h,cpp} to enable
		# the HID-based macOS path. Defined as PUBLIC so that consumers (qCC,
		# ccViewer) also see it when compiling their mainwindow.cpp files.
		target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_MAC_HID )

		if( OPTION_HID_DEBUG )
			target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_HID_DEBUG )
		endif()

		target_link_libraries( ${PROJECT_NAME} PUBLIC hidapi::hidapi )
		if ( WIN32 )
			#set( HIDAPI_LIB ${HIDAPI_BINARY_DIR}/src/windows/hidapi.dll )
			#install( PROGRAMS ${HIDAPI_LIB} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} )
		elseif ( LINUX )
			set( HIDAPI_LIB ${HIDAPI_BINARY_DIR}/src/linux/libhidapi-hidraw.so.0.16.0 )
			install( FILES ${HIDAPI_LIB} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} RENAME "libhidapi-hidraw.so.0")
		endif()	
	endif()

	function( target_link_3DXWARE ) # 1 argument: ARGV0 = target
		# nothing to do on macOS (hidapi is linked via the target itself)
	endfunction()
endif()
