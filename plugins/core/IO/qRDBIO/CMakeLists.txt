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


	option( PLUGIN_IO_QRDB_INSTALL_DEPENDENCY "Install RDB shared library to library folder" OFF)
	if( PLUGIN_IO_QRDB_INSTALL_DEPENDENCY )
		if(MSVC)
			# on Windows put the dll to the binaries for them to find them
			set(_rdb_libdirs "${CLOUDCOMPARE_DEST_FOLDER}")
			if (${OPTION_BUILD_CCVIEWER})
				list(APPEND _rdb_libdirs "${CCVIEWER_DEST_FOLDER}")
			endif()
		elseif(APPLE)
			set(_rdb_libdirs CLOUDCOMPARE_MAC_FRAMEWORK_DIR)
		else()
			# some distros prefer "lib" others like more specific folders like "lib64" (handled by GNUInstallDirs)
			if( CMAKE_INSTALL_LIBDIR STREQUAL "" )
				# see: https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
				include(GNUInstallDirs)
			endif()

			if( NOT CMAKE_INSTALL_LIBDIR OR CMAKE_INSTALL_LIBDIR STREQUAL "" )
				# still no library destination? just use "lib" to be extra safe
				set(_rdb_libdirs "lib")
			else()
				set(_rdb_libdirs "${CMAKE_INSTALL_LIBDIR}")
			endif()
		endif()
		get_target_property(RDB_LIBRARY_FILE rdbc IMPORTED_LOCATION)
		# resolve possible symlink and install the real library
		get_filename_component(lib_REAL ${RDB_LIBRARY_FILE} REALPATH)
		foreach (dest ${_rdb_libdirs})
			_InstallFiles(FILES ${lib_REAL} DEST_PATH ${dest})
		endforeach()
		if( NOT WIN32 )
			# find all the symlinks linking to lib_REAL
			string(REGEX MATCH "\\.so" is_shared_lib_linux "${RDB_LIBRARY_FILE}")
			string(REGEX MATCH "\\.dylib$" is_shared_lib_darwin "${RDB_LIBRARY_FILE}")
			if(is_shared_lib_linux OR is_shared_lib_darwin)
				if(is_shared_lib_linux)
					string(REGEX REPLACE "\\.so.*" ".so" lib_base "${RDB_LIBRARY_FILE}")
					file(GLOB lib_symlinks "${lib_base}*")
				else()
					string(REGEX REPLACE "\\.dylib$" "" lib_base "${RDB_LIBRARY_FILE}")
					file(GLOB lib_symlinks "${lib_base}*.dylib")
				endif()
				foreach(lib_sym ${lib_symlinks})
					STRING(COMPARE NOTEQUAL "${lib_sym}" "${lib_REAL}" real_lib_different)
					if(NOT real_lib_different)
						continue() # we found the real lib, not a symlink, skip it
					endif()
					# actually install the found symlink
					foreach (dest ${_rdb_libdirs})
						_InstallFiles(FILES ${lib_sym} DEST_PATH ${dest})
					endforeach()
				endforeach()
			else() # neither linux nor darwin,
				# no special symlink handling implemented, just install if different paths to prevent doulbe installation
				STRING(COMPARE NOTEQUAL "${RDB_LIBRARY_FILE}" "${lib_REAL}" real_lib_different)
				if(real_lib_different)
					# only install if different
					foreach (dest ${_rdb_libdirs})
						_InstallFiles(FILES ${RDB_LIBRARY_FILE} DEST_PATH ${dest})
					endforeach()
				endif()
			endif()
		endif() # NOT WIN32
	endif() # install dependency
endif()
