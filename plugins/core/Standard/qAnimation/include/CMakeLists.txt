
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qAnimation.h
		${CMAKE_CURRENT_LIST_DIR}/qAnimationDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ViewInterpolate.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
