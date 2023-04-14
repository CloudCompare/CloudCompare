
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccTreeIsoDlg.h
		${CMAKE_CURRENT_LIST_DIR}/TreeIso.h
		${CMAKE_CURRENT_LIST_DIR}/TreeIsoHelper.h
		${CMAKE_CURRENT_LIST_DIR}/qTreeIso.h
		${CMAKE_CURRENT_LIST_DIR}/qTreeIsoCommands.h		
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
