
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/E57Header.h
        ${CMAKE_CURRENT_LIST_DIR}/E57Filter.h
        ${CMAKE_CURRENT_LIST_DIR}/qE57IO.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
