
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/PcdFilter.h
		${CMAKE_CURRENT_LIST_DIR}/qPclIO.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}
)
