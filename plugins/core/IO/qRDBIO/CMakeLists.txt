option( PLUGIN_IO_QRDB "Install RDB 2 I/O plugin" OFF )
if( PLUGIN_IO_QRDB )
	option( PLUGIN_IO_QRDB_FETCH_DEPENDENCY "Fetch RDB SDK from public server" OFF)
	if (PLUGIN_IO_QRDB_FETCH_DEPENDENCY)
		# download current version number so the download URL can be set accordingly
		file(DOWNLOAD "https://repository.riegl.com/software/libraries/rdblib/current/version.txt" "${CMAKE_CURRENT_BINARY_DIR}/rdb_version.txt" STATUS DOWNLOAD_RESULT)
		file(STRINGS "${CMAKE_CURRENT_BINARY_DIR}/rdb_version.txt" _RDB_VERSION)
		# set download URL OS/Architecture specific (only x86/x86_64 handled right now)
		set(_RDB_URL_BASE "https://repository.riegl.com/software/libraries/rdblib/current")
		if ( WIN32 )
			if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64bit build
				set(_RDB_URL "${_RDB_URL_BASE}/windows/rdblib-${_RDB_VERSION}-x86_64-windows.zip")
			elseif (CMAKE_SIZEOF_VOID_P EQUAL 4) # 32bit build
				set(_RDB_URL "${_RDB_URL_BASE}/windows/rdblib-${_RDB_VERSION}-x86-windows.zip")
			else()
				message(FATAL_ERROR "PLUGIN_IO_QRDB: unhandled Platform Architecture to fetch RDB SDK")
			endif()
		elseif( APPLE )
			if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64bit build
				set(_RDB_URL "${_RDB_URL_BASE}/macos/rdblib-${_RDB_VERSION}-x86_64-darwin.tar.gz")
			else()
				message(FATAL_ERROR "PLUGIN_IO_QRDB: unhandled Platform Architecture to fetch RDB SDK")
			endif()
		elseif( UNIX )
			if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64bit build
				set(_RDB_URL "${_RDB_URL_BASE}/linux/rdblib-${_RDB_VERSION}-x86_64-linux.tar.gz")
			elseif (CMAKE_SIZEOF_VOID_P EQUAL 4) # 32bit build
				set(_RDB_URL "${_RDB_URL_BASE}/linux/rdblib-${_RDB_VERSION}-x86-linux.tar.gz")
			else()
				message(FATAL_ERROR "PLUGIN_IO_QRDB: unhandled Platform Architecture to fetch RDB SDK")
			endif()
		else()
			message(FATAL_ERROR "PLUGIN_IO_QRDB: unhandled Platform to fetch RDB SDK")
		endif()

		# download the SDK and set rdb_DIR for later find_package() call
		message(STATUS "PLUGIN_IO_QRDB: downloading rdb_sdk version '${_RDB_VERSION}' from URL '${_RDB_URL}'")
		include(FetchContent)
		FetchContent_Declare(rdb_sdk
			URL "${_RDB_URL}")
		FetchContent_GetProperties(rdb_sdk)
		if (NOT rdb_sdk_POPULATED)
			FetchContent_Populate(rdb_sdk)
			set(rdb_DIR "${rdb_sdk_SOURCE_DIR}/interface/cpp")
			message(STATUS "PLUGIN_IO_QRDB: populated rdb_sdk and setting 'rdb_DIR' to '${rdb_DIR}'")
		endif()
	endif()

	project( QRDB_IO_PLUGIN )
	
	find_package( rdb REQUIRED )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	target_link_libraries( ${PROJECT_NAME} rdbcpp )
endif()
