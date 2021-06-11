target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/CC
		${CMAKE_CURRENT_SOURCE_DIR}/libs
		${CMAKE_CURRENT_SOURCE_DIR}/qCC
		${CMAKE_CURRENT_SOURCE_DIR}/generalUtility
)



add_subdirectory( CC )
add_subdirectory( libs )
add_subdirectory( qCC )
add_subdirectory( generalUtility )