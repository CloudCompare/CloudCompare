# CC custom CMake script for QCustomPlot
# https://www.qcustomplot.com/

project( QCustomPlot )

qt_add_library( ${PROJECT_NAME} STATIC )

target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qcustomplot.cpp
		${CMAKE_CURRENT_LIST_DIR}/qcustomplot.h
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties( ${PROJECT_NAME}
	PROPERTIES POSITION_INDEPENDENT_CODE ON
	CXX_VISIBILITY_PRESET hidden
)

target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		Qt6::Core
		Qt6::Gui
		Qt6::Widgets
)
