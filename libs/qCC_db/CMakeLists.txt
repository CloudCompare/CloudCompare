project( QCC_DB_LIB )

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_link_libraries( ${PROJECT_NAME}
    CCCoreLib
    CC_FBO_LIB
    Qt5::Widgets
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	CXX_VISIBILITY_PRESET hidden
)

# install (shared) lib to specified destinations
if( WIN32 OR APPLE )
	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()
