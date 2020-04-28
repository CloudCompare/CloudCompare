# InstallSharedLibrary should be called once for each shared library in the libs directory.
# This function installs the shared library in the correct places for each platform.
# For macOS and Windows, it puts them with each application (CloudCompare and ccViewer).
# For Linux, it puts it in the proper place for shared libraries so both applcations can
# access them.
#
# Arguments:
#   TARGET The name of the library target
function( InstallSharedLibrary )
    cmake_parse_arguments(
            INSTALL_SHARED_LIB
            ""
            "TARGET"
            ""
            ${ARGN}
        )

	message( STATUS "Install shared library: ${INSTALL_SHARED_LIB_TARGET}")

	if( WIN32 OR APPLE )
		foreach( DEST ${INSTALL_DESTINATIONS} )
			install_shared( ${INSTALL_SHARED_LIB_TARGET} ${DEST} 1 )
		endforeach()
	else()
		install_shared( ${INSTALL_SHARED_LIB_TARGET} ${CMAKE_INSTALL_LIBDIR}/cloudcompare )
	endif()
endfunction()
