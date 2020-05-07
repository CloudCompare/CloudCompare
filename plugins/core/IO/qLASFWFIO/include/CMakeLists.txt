
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LASFWFFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/qLASFWFIO.h
        ${CMAKE_CURRENT_LIST_DIR}/qLASFWFIOCommands.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
