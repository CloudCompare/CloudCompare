# ------------------------------------------------------------------------------
# Dxf Lib + CMake support for CloudCompare
# ------------------------------------------------------------------------------

OPTION( OPTION_USE_PCATPS_LIB "Build with pc_atps Lib (plane segmentation support)" ON )
if( ${OPTION_USE_PCATPS_LIB} )
	# set( PCATPS_LIB_DIR )
	set( PCATPS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/contrib/PC_ATPS" CACHE PATH "PC_ATPS root path")
	set( PCATPS_INCLUDE_DIR "${PCATPS_ROOT}" CACHE PATH "PC_ATPS include path")
	set( PCATPS_LIBRARY_DEBUG "${PCATPS_ROOT}/debug/PC_ATPS.lib"  CACHE FILEPATH "PC_ATPS debug lib path" )
	set( PCATPS_LIBRARY_RELEASE "${PCATPS_ROOT}/release/PC_ATPS.lib"  CACHE FILEPATH "PC_ATPS release lib path" )
	
endif()

# Link project with dxflib library
function( target_link_PCATPS )
	if( ${OPTION_USE_PCATPS_LIB} )
		target_include_directories( ${PROJECT_NAME} PUBLIC ${PCATPS_INCLUDE_DIR})
		target_link_libraries( ${PROJECT_NAME} debug ${PCATPS_LIBRARY_DEBUG} optimized ${PCATPS_LIBRARY_RELEASE})	
		set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS PCATPS_SUPPORT )

		file(COPY ${PCATPS_ROOT}/debug/PC_ATPS.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/debug/)
		install( FILES ${PCATPS_ROOT}/debug/PC_ATPS.dll CONFIGURATIONS Debug DESTINATION ${CLOUDCOMPARE_DEST_FOLDER}_debug)
		
		file(COPY ${PCATPS_ROOT}/release/PC_ATPS.dll DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/release/)
		install( FILES ${PCATPS_ROOT}/release/PC_ATPS.dll CONFIGURATIONS Release DESTINATION ${CLOUDCOMPARE_DEST_FOLDER} )		

	endif()
endfunction()