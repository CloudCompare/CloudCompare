
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/cameraParamDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/displaySettingsDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/pickOneElementDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/stereoModeDlg.ui
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_LIST_DIR}
)
