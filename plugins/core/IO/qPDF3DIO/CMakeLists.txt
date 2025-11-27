# CMakeLists.txt for qPDF3DIO plugin
# ===================================
#
# This CMakeLists.txt file configures the build system for the 3D PDF export plugin.
# The plugin provides functionality to export point clouds and meshes as interactive
# 3D PDF files.
#
# Dependencies:
# - Qt framework (QtCore, QtGui)
# - CloudCompare core libraries
# - PDF generation library (to be added)
# - U3D/PRC format library (to be added)
#
# Build Configuration:
# - Plugin is built as a shared library
# - Automatically loaded by CloudCompare at runtime
# - Requires proper plugin metadata (info.json)
#

# Project name
project( qPDF3DIO )

# Minimum CMake version
cmake_minimum_required( VERSION 3.10 )

# Plugin source files
set( PDF3DIO_SOURCES
	src/qPDF3DIO.cpp
	src/PDF3DFilter.cpp
	qPDF3DIO.qrc
)

# Plugin header files
set( PDF3DIO_HEADERS
	include/qPDF3DIO.h
	include/PDF3DFilter.h
)

# Create plugin library
add_library( qPDF3DIO SHARED
	${PDF3DIO_SOURCES}
	${PDF3DIO_HEADERS}
)

# Set target properties
set_target_properties( qPDF3DIO PROPERTIES
	PREFIX ""
	OUTPUT_NAME "qPDF3DIO"
)

# Link libraries
target_link_libraries( qPDF3DIO
	PUBLIC
		qCC_db
		qCC_io
		CCPluginAPI
		Qt6::Core
		Qt6::Gui
)

# Include directories
target_include_directories( qPDF3DIO
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Install plugin
install( TARGETS qPDF3DIO
	LIBRARY DESTINATION ${CC_PLUGINS_DIRECTORY_ROOT}
)

# Note: Future dependencies for PDF generation will be added here
# For example:
# find_package( PoDoFo )
# target_link_libraries( qPDF3DIO PRIVATE PoDoFo::PoDoFo )

