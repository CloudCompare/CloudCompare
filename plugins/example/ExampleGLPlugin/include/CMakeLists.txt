
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Bilateral.h
        ${CMAKE_CURRENT_LIST_DIR}/ExampleGLPlugin.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
