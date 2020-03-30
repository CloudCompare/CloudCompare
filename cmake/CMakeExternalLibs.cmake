# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------
## we will use cmake automoc / autouic / autorcc feature
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set( QT5_ROOT_PATH CACHE PATH "Qt5 root directory (i.e. where the 'bin' folder lies)" )
if ( QT5_ROOT_PATH )
	list( APPEND CMAKE_PREFIX_PATH ${QT5_ROOT_PATH} )
endif()

# find qt5 components
# find_package(Qt5 COMPONENTS OpenGL Widgets Core Gui PrintSupport Concurrent REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5OpenGLExtensions REQUIRED)
find_package(Qt5Svg REQUIRED)

# in the case no Qt5Config.cmake file could be found, cmake will explicitly ask the user for the QT5_DIR containing it!
# thus no need to keep additional variables and checks

# Starting with the QtCore lib, find the bin and root directories
get_target_property(QT5_LIB_LOCATION Qt5::Core LOCATION_${CMAKE_BUILD_TYPE})
get_filename_component(QT_BINARY_DIR ${QT5_LIB_LOCATION} DIRECTORY)

# Apple uses frameworks - move up until we get to the base directory to set the bin directory properly
if ( APPLE )
	get_filename_component(QT_BINARY_DIR ${QT_BINARY_DIR} DIRECTORY)
	get_filename_component(QT_BINARY_DIR ${QT_BINARY_DIR} DIRECTORY)
	set(QT_BINARY_DIR "${QT_BINARY_DIR}/bin")	

	set( MACDEPLOYQT "${QT_BINARY_DIR}/macdeployqt" )
endif()

# set QT5_ROOT_PATH if it wasn't set by the user
if ( NOT QT5_ROOT_PATH )
	get_filename_component(QT5_ROOT_PATH ${QT_BINARY_DIR} DIRECTORY)
endif()

include_directories(${Qt5OpenGL_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Core_INCLUDE_DIRS}
                    ${Qt5Gui_INCLUDE_DIRS}
                    ${Qt5Concurrent_INCLUDE_DIRS}
                    ${Qt5PrintSupport_INCLUDE_DIRS}
					)

# turn on QStringBuilder for more efficient string construction
#	see https://doc.qt.io/qt-5/qstring.html#more-efficient-string-construction
add_definitions( -DQT_USE_QSTRINGBUILDER )
				

# ------------------------------------------------------------------------------
# OpenGL
# ------------------------------------------------------------------------------
if ( MSVC )
	# Where to find OpenGL libraries
	set(WINDOWS_OPENGL_LIBS "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64" CACHE PATH "WindowsSDK libraries" )
	list( APPEND CMAKE_PREFIX_PATH ${WINDOWS_OPENGL_LIBS} )
endif()
				
# ------------------------------------------------------------------------------
# OpenMP
# ------------------------------------------------------------------------------
if ( NOT APPLE )
	find_package(OpenMP QUIET)
	if (OPENMP_FOUND)
		message(STATUS "OpenMP found")
		set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	endif()
endif()

# Intel's Threading Building Blocks (TBB)
if (COMPILE_CC_CORE_LIB_WITH_TBB)
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_TBB")
	include_directories( ${TBB_INCLUDE_DIRS} )
endif()
