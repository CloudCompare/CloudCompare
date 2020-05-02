
target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/CCPluginAPI.h
		${CMAKE_CURRENT_LIST_DIR}/ccCommandLineInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccMainAppInterface.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
