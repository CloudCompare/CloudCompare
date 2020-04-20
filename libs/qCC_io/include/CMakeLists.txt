
target_sources( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/ShpFilter.h
		${CMAKE_CURRENT_LIST_DIR}/AsciiFilter.h
		${CMAKE_CURRENT_LIST_DIR}/AsciiOpenDlg.h
		${CMAKE_CURRENT_LIST_DIR}/AsciiSaveDlg.h
		${CMAKE_CURRENT_LIST_DIR}/BinFilter.h
		${CMAKE_CURRENT_LIST_DIR}/ccGlobalShiftManager.h
		${CMAKE_CURRENT_LIST_DIR}/ccShiftAndScaleCloudDlg.h
		${CMAKE_CURRENT_LIST_DIR}/DepthMapFileFilter.h
		${CMAKE_CURRENT_LIST_DIR}/DxfFilter.h
		${CMAKE_CURRENT_LIST_DIR}/FileIO.h
		${CMAKE_CURRENT_LIST_DIR}/FileIOFilter.h
		${CMAKE_CURRENT_LIST_DIR}/ImageFileFilter.h
		${CMAKE_CURRENT_LIST_DIR}/PlyFilter.h
		${CMAKE_CURRENT_LIST_DIR}/PlyOpenDlg.h
		${CMAKE_CURRENT_LIST_DIR}/qCC_io.h
		${CMAKE_CURRENT_LIST_DIR}/RasterGridFilter.h
		${CMAKE_CURRENT_LIST_DIR}/rply.h
		${CMAKE_CURRENT_LIST_DIR}/ShpDBFFields.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
