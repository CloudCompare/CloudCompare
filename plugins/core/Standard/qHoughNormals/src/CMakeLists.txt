
set( CC_PLUGIN_CUSTOM_HEADER_LIST
	${CC_PLUGIN_CUSTOM_HEADER_LIST} 
	${CMAKE_CURRENT_SOURCE_DIR}/nanoflann/include/nanoflann.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/normals_Hough/Normals.h # their cmake file does not add it
	${CMAKE_CURRENT_SOURCE_DIR}/qHoughNormalsDialog.h
	PARENT_SCOPE
)

set( CC_PLUGIN_CUSTOM_SOURCE_LIST
	${CC_PLUGIN_CUSTOM_SOURCE_LIST} 
	${CMAKE_CURRENT_SOURCE_DIR}/qHoughNormalsDialog.cpp
	PARENT_SCOPE
)
