
target_sources( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/saveSHPFileDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/saveAsciiFileDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/openPlyFileDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/openAsciiFileDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/importDBFFieldDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/globalShiftAndScaleDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/globalShiftAndScaleAboutDlg.ui
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
	AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_LIST_DIR}
)
