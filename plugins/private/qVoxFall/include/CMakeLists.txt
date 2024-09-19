
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qVoxFall.h
		${CMAKE_CURRENT_LIST_DIR}/qVoxFallDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qVoxFallDisclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qVoxFallProcess.h
		${CMAKE_CURRENT_LIST_DIR}/qVoxFallTools.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
