
target_sources( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}/BaseFilter.h
		${CMAKE_CURRENT_LIST_DIR}/ExtractSIFT.h
		${CMAKE_CURRENT_LIST_DIR}/MLSSmoothingUpsampling.h
		${CMAKE_CURRENT_LIST_DIR}/NormalEstimation.h
		${CMAKE_CURRENT_LIST_DIR}/StatisticalOutliersRemover.h
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/BaseFilter.cpp
		${CMAKE_CURRENT_LIST_DIR}/ExtractSIFT.cpp
		${CMAKE_CURRENT_LIST_DIR}/MLSSmoothingUpsampling.cpp
		${CMAKE_CURRENT_LIST_DIR}/NormalEstimation.cpp
		${CMAKE_CURRENT_LIST_DIR}/StatisticalOutliersRemover.cpp
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory( dialogs )

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
	    AUTOUIC ON # FIXME Remove after everything has moved to targets and we can set it globally
		AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_LIST_DIR}/dialogs
)
