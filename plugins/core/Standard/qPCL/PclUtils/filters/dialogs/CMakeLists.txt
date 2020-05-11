
target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/MLSDialog.h
		${CMAKE_CURRENT_LIST_DIR}/NormalEstimationDlg.h
		${CMAKE_CURRENT_LIST_DIR}/SIFTExtractDlg.h
		${CMAKE_CURRENT_LIST_DIR}/StatisticalOutliersRemoverDlg.h
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/MLSDialog.cpp
		${CMAKE_CURRENT_LIST_DIR}/NormalEstimationDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/SIFTExtractDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/StatisticalOutliersRemoverDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/MLSDialog.ui
		${CMAKE_CURRENT_LIST_DIR}/NormalEstimationDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/SIFTExtractDlg.ui
		${CMAKE_CURRENT_LIST_DIR}/StatisticalOutliersRemoverDlg.ui
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}
)
