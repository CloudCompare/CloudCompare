
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/CCPluginAPI.h
		${CMAKE_CURRENT_LIST_DIR}/ccColorScaleSelector.h
		${CMAKE_CURRENT_LIST_DIR}/ccColorScaleEditorDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ccColorScaleEditorWidget.h
		${CMAKE_CURRENT_LIST_DIR}/ccCommandLineInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccMainAppInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccOverlayDialog.h
		${CMAKE_CURRENT_LIST_DIR}/ccPersistentSettings.h
		${CMAKE_CURRENT_LIST_DIR}/ccPickingHub.h
		${CMAKE_CURRENT_LIST_DIR}/ccPickingListener.h
		${CMAKE_CURRENT_LIST_DIR}/ccQtHelpers.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}
)
