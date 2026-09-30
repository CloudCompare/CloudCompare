if(WIN32)
	option( OPTION_SUPPORT_3DMOUSE_WITH_3DxWARE "Build with 3DxWare (3D mouse support)" OFF )
endif()
option( OPTION_SUPPORT_3DMOUSE_WITH_HIDAPI "Build with hidapi (3D mouse support)" OFF )

if( OPTION_SUPPORT_3DMOUSE_WITH_3DxWARE ) # former 3DxWare SDK (Windows only)

	if( OPTION_SUPPORT_3DMOUSE_WITH_HIDAPI )
		message( WARNING "3DxWARE option takes precedence over HIDAPI")
	endif()

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

	target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_3DMOUSE_SUPPORT )

	function( target_link_3DxWARE ) # 1 argument: ARGV0 = target
		if( OPTION_SUPPORT_3DMOUSE_WITH_3DxWARE )
			target_link_libraries( ${ARGV0} PUBLIC optimized ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmath.lib)
			if ( CMAKE_CONFIGURATION_TYPES )
				target_link_libraries( ${ARGV0} PUBLIC debug ${3DXWARE_LIB_DIR}/siapp.lib ${3DXWARE_LIB_DIR}/spwmathD.lib )
			endif()
		endif()
	endfunction()

elseif( OPTION_SUPPORT_3DMOUSE_WITH_HIDAPI )

	find_package( hidapi 0.13.0 CONFIG )

	# hidapi-based 3DConnexion support, originally for macOS but also works on Linux and Windows (no more 3DxWare SDK required)
	# We drive the 3DConnexion device directly via hidapi (no 3DxWare SDK).
	# The hidapi sources are pulled in as a Git submodule under extern/hidapi.

	if( NOT hidapi_FOUND )
		message( STATUS "[3DMouse] HIDAPI Lib not found, using vendored HIDAPI" )
		set( HIDAPI_WITH_TESTS OFF CACHE BOOL "" FORCE )
		if( LINUX )
			set( HIDAPI_WITH_HIDRAW ON CACHE BOOL "" FORCE )
			set( HIDAPI_WITH_LIBUSB OFF CACHE BOOL "" FORCE )
		endif()
		# Build the hidapi submodule. The path to the source and binary directories is relative to the current CMakeLists.txt file
		# (useful when CloudCompare is a git submodule in another project).
		set( CURRENT_BUILD_SHARED_LIBS "${BUILD_SHARED_LIBS}" )
		set( BUILD_SHARED_LIBS OFF )
		add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/extern/hidapi EXCLUDE_FROM_ALL)
		set( BUILD_SHARED_LIBS "${CURRENT_BUILD_SHARED_LIBS}" )
	endif()

	# Allow target_link_libraries() to act on targets defined in the parent directory.
	cmake_policy( SET CMP0079 NEW )

	# Optional verbose logging of raw HID reports to the CloudCompare console.
	# Off by default; turn on to diagnose device-specific report formats.
	option( OPTION_HID_DEBUG "Enable verbose HID report logging (3D mouse)" OFF )

	target_sources( ${PROJECT_NAME}
			PRIVATE
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.h
				${CMAKE_CURRENT_LIST_DIR}/cc3DMouseManager.cpp
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.h
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput.cpp
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput_hid.h
				${CMAKE_CURRENT_LIST_DIR}/Mouse3DInput_hid.cpp
		)

	target_include_directories( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR} )

	target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_3DMOUSE_SUPPORT )

	# Used by Mouse3DInput.{h,cpp} and Mouse3DInput_hid.{h,cpp} to enable
	# the HID-based path. Defined as PUBLIC so that consumers (qCC,
	# ccViewer) also see it when compiling their mainwindow.cpp files.
	target_compile_definitions( ${PROJECT_NAME} PUBLIC CC_3DMOUSE_HID )

	if( OPTION_HID_DEBUG )
		target_compile_definitions( ${PROJECT_NAME} PRIVATE CC_HID_DEBUG )
	endif()

	target_link_libraries( ${PROJECT_NAME} PUBLIC hidapi::hidapi )
endif()
