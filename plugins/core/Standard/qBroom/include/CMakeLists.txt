
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qBroom.h
		${CMAKE_CURRENT_LIST_DIR}/qBroomDisclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qBroomDlg.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
