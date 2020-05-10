
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccPointDescriptor.h
		${CMAKE_CURRENT_LIST_DIR}/classifier.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupo.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupo2DViewDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoClassifDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoCommands.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoDisclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoProcess.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoTools.h
		${CMAKE_CURRENT_LIST_DIR}/qCanupoTrainingDialog.h
		${CMAKE_CURRENT_LIST_DIR}/trainer.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
