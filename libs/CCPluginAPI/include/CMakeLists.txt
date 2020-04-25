
target_sources( CCPluginAPI
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}/ccStdPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccCommandLineInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccDefaultPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccGLPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccIOPluginInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccMainAppInterface.h
		${CMAKE_CURRENT_LIST_DIR}/ccPluginInterface.h
)

target_include_directories( CCPluginAPI
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
