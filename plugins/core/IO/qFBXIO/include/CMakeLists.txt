
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/FBXCommand.h
        ${CMAKE_CURRENT_LIST_DIR}/FBXFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/qFBXIO.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
