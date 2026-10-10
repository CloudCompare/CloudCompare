# Include directory CMakeLists.txt

target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/FFDPlugin.h
		${CMAKE_CURRENT_LIST_DIR}/FFDLattice.h
		${CMAKE_CURRENT_LIST_DIR}/FFDAction.h
		${CMAKE_CURRENT_LIST_DIR}/FFDLatticeParamsDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ccFFDLatticeDisplay.h
		${CMAKE_CURRENT_LIST_DIR}/ccFFDDeformationTool.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
