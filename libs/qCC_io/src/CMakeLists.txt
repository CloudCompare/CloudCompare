
target_sources( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/rply.c
		${CMAKE_CURRENT_LIST_DIR}/AsciiFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/AsciiOpenDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/AsciiSaveDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/BinFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGlobalShiftManager.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccShiftAndScaleCloudDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/DepthMapFileFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/DxfFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/FileIO.cpp
		${CMAKE_CURRENT_LIST_DIR}/FileIOFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/ImageFileFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/PlyFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/PlyOpenDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/RasterGridFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/ShpDBFFields.cpp
		${CMAKE_CURRENT_LIST_DIR}/ShpFilter.cpp
)
