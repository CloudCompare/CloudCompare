
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccMapWindow.h
		${CMAKE_CURRENT_LIST_DIR}/ccSymbolCloud.h
		${CMAKE_CURRENT_LIST_DIR}/distanceMapGenerationDlg.h
		${CMAKE_CURRENT_LIST_DIR}/distanceMapGenerationTool.h
		${CMAKE_CURRENT_LIST_DIR}/dxfProfilesExportDlg.h
		${CMAKE_CURRENT_LIST_DIR}/dxfProfilesExporter.h
		${CMAKE_CURRENT_LIST_DIR}/profileImportDlg.h
		${CMAKE_CURRENT_LIST_DIR}/profileLoader.h
		${CMAKE_CURRENT_LIST_DIR}/qSRA.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
