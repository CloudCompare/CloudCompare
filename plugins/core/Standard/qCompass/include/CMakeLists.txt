
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccCompass.h
		${CMAKE_CURRENT_LIST_DIR}/ccCompassDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ccCompassExport.h
		${CMAKE_CURRENT_LIST_DIR}/ccCompassImport.h
		${CMAKE_CURRENT_LIST_DIR}/ccCompassInfo.h
		${CMAKE_CURRENT_LIST_DIR}/ccFitPlane.h
		${CMAKE_CURRENT_LIST_DIR}/ccFitPlaneTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccGeoObject.h
		${CMAKE_CURRENT_LIST_DIR}/ccLineation.h
		${CMAKE_CURRENT_LIST_DIR}/ccLineationTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccMapDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ccMeasurement.h
		${CMAKE_CURRENT_LIST_DIR}/ccMouseCircle.h
		${CMAKE_CURRENT_LIST_DIR}/ccNote.h
		${CMAKE_CURRENT_LIST_DIR}/ccNoteTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccPinchNode.h
		${CMAKE_CURRENT_LIST_DIR}/ccPinchNodeTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccPointPair.h
		${CMAKE_CURRENT_LIST_DIR}/ccThickness.h
		${CMAKE_CURRENT_LIST_DIR}/ccThicknessTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccTopologyRelation.h
		${CMAKE_CURRENT_LIST_DIR}/ccTopologyTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccTrace.h
		${CMAKE_CURRENT_LIST_DIR}/ccTraceTool.h
		${CMAKE_CURRENT_LIST_DIR}/ccSNECloud.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
