
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/cellsFusionDlg.h
		${CMAKE_CURRENT_LIST_DIR}/classificationParamsDlg.h
		${CMAKE_CURRENT_LIST_DIR}/disclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/facetsClassifier.h
		${CMAKE_CURRENT_LIST_DIR}/facetsExportDlg.h
		${CMAKE_CURRENT_LIST_DIR}/fastMarchingForFacetExtraction.h
		${CMAKE_CURRENT_LIST_DIR}/kdTreeForFacetExtraction.h
		${CMAKE_CURRENT_LIST_DIR}/qFacets.h
		${CMAKE_CURRENT_LIST_DIR}/stereogramDlg.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
