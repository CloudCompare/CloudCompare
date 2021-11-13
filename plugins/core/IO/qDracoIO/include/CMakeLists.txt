
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/DRCFilter.h
		${CMAKE_CURRENT_LIST_DIR}/qDracoIO.h
		${CMAKE_CURRENT_LIST_DIR}/SaveDracoFileDlg.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
