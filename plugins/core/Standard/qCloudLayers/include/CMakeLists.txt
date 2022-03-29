
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qCloudLayers.h
		${CMAKE_CURRENT_LIST_DIR}/ccMouseCircle.h
		${CMAKE_CURRENT_LIST_DIR}/ccCloudLayersDlg.h
		${CMAKE_CURRENT_LIST_DIR}/ccAsprsModel.h
		${CMAKE_CURRENT_LIST_DIR}/ccColorStyledDelegate.h
		${CMAKE_CURRENT_LIST_DIR}/ccCloudLayersHelper.h

)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
