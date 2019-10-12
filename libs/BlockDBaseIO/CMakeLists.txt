cmake_minimum_required(VERSION 3.0)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )



include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

project( libBlockDBaseIO )

file( GLOB header_list *.h *.hpp)
file( GLOB source_list *.cpp *.c)

set( CMAKE_CXX_VISIBILITY_PRESET hidden )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list}  )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libs") 

if(PostgreSQL_FOUND)
	target_link_libraries(${PROJECT_NAME} PUBLIC ${PostgreSQL_LIBRARIES})	
	include_directories( ${PostgreSQL_INCLUDE_DIRS} )
endif()

if( APPLE )
	set_target_properties( ${PROJECT_NAME} PROPERTIES
		INSTALL_RPATH "." )
endif()

# install (shared) lib to specified destinations
if( WIN32 OR APPLE )
	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()
