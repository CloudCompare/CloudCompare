
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/FFDPlugin.cpp
		${CMAKE_CURRENT_LIST_DIR}/FFDLattice.cpp
		${CMAKE_CURRENT_LIST_DIR}/FFDAction.cpp
		${CMAKE_CURRENT_LIST_DIR}/FFDLatticeParamsDlg.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccFFDLatticeDisplay.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccFFDDeformationTool.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccFFDDeformationApplier.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccSelectionRectangle.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/../info.json
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/../include
		${CMAKE_SOURCE_DIR}/libs/qCC_glWindow/include
)
