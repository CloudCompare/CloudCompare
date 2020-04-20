
target_sources( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/ccShader.h
		${CMAKE_CURRENT_LIST_DIR}/ccGlFilter.h
		${CMAKE_CURRENT_LIST_DIR}/ccFrameBufferObject.h
		${CMAKE_CURRENT_LIST_DIR}/ccBilateralFilter.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
