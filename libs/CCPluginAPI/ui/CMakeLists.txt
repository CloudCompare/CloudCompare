
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/colorScaleEditorDlg.ui
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_LIST_DIR}
)
