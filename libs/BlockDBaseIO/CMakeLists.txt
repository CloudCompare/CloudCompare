cmake_minimum_required(VERSION 3.0)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

project( libBlockDBaseIO )

file( GLOB header_list *.h *.hpp)
file( GLOB source_list *.cpp *.c)

set( CMAKE_CXX_VISIBILITY_PRESET hidden )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list}  )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libs") 
add_definitions(-DUSE_LASPRJIO_AS_LIB)
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
	if(PostgreSQL_FOUND)
		SET(PostgreSQL_BIN_DIRS ${PostgreSQL_LIBRARY_DIRS}/../bin CACHE PATH "folder path to libpq.dll, etc.")
		SET (PostgreSQL_DLLs 
		${PostgreSQL_BIN_DIRS}/libpq.dll 
		${PostgreSQL_BIN_DIRS}/libintl-9.dll 
		${PostgreSQL_BIN_DIRS}/libiconv-2.dll 
		${PostgreSQL_BIN_DIRS}/libwinpthread-1.dll)
		foreach( dest ${INSTALL_DESTINATIONS} )
			copy_files("${PostgreSQL_DLLs}" ${dest} )
		endforeach()
		foreach(dll ${PostgreSQL_DLLs})		
			file(COPY ${dll} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/debug/)
			file(COPY ${dll} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/release/)		
		endforeach()
	endif()

	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()
