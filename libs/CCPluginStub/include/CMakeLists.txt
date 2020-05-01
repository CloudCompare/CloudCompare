
target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/ccDefaultPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccGLPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccIOPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccStdPluginInterface.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
