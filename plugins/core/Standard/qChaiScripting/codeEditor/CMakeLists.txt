target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/chaiScriptCodeEditorMainWindow.cpp
		${CMAKE_CURRENT_LIST_DIR}/codeEditor.cpp
		${CMAKE_CURRENT_LIST_DIR}/chaiScriptSyntaxHighlighter.cpp
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)

