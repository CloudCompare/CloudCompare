
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ccXRayRenderer.h
		${CMAKE_CURRENT_LIST_DIR}/ccXRayRenderer.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccXRaySettings.cpp
	    ${CMAKE_CURRENT_LIST_DIR}/ccRenderingTools.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGLWindow.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGLWindowInterface.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGLWindowSignalEmitter.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGLWindowStereo.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGuiParameters.cpp
		${CMAKE_CURRENT_LIST_DIR}/ccGLUtils.cpp
)
