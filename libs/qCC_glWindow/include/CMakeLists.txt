
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccGLUtils.h
		${CMAKE_CURRENT_LIST_DIR}/ccGLWidget.h
		${CMAKE_CURRENT_LIST_DIR}/ccGLWindow.h
		${CMAKE_CURRENT_LIST_DIR}/ccGuiParameters.h
		${CMAKE_CURRENT_LIST_DIR}/ccRenderingTools.h
		${CMAKE_CURRENT_LIST_DIR}/qCC_glWindow.h
)

if ( OPTION_USE_OCULUS_SDK )
	target_sources( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_LIST_DIR}/oculus/ccOculus.h
	)
endif()

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}
)
