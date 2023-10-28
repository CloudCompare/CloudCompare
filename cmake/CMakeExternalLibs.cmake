# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------

set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTORCC ON )

# FIXME Eventually turn this on when we've completed the move to targets
#set( CMAKE_AUTOUIC ON )

set( QT5_ROOT_PATH CACHE PATH "Qt5 root directory (i.e. where the 'bin' folder lies)" )
if ( QT5_ROOT_PATH )
	list( APPEND CMAKE_PREFIX_PATH ${QT5_ROOT_PATH} )
endif()

find_package( Qt5
    COMPONENTS
        Concurrent
        Core
        Gui
        OpenGL
        OpenGLExtensions
        PrintSupport
        Svg
        Widgets
    REQUIRED
)

# in the case no Qt5Config.cmake file could be found, cmake will explicitly ask the user for the QT5_DIR containing it!
# thus no need to keep additional variables and checks

# Starting with the QtCore lib, find the bin and root directories
get_target_property( Qt5_LIB_LOCATION Qt5::Core LOCATION_${CMAKE_BUILD_TYPE} )
get_filename_component( Qt5_LIB_LOCATION ${Qt5_LIB_LOCATION} DIRECTORY )

if ( WIN32 )
    get_target_property( QMAKE_LOCATION Qt5::qmake IMPORTED_LOCATION )
    get_filename_component( Qt5_BIN_DIR ${QMAKE_LOCATION} DIRECTORY )
    get_filename_component( QT5_ROOT_PATH "${Qt5_BIN_DIR}/.." ABSOLUTE )
endif()

# turn on QStringBuilder for more efficient string construction
#	see https://doc.qt.io/qt-5/qstring.html#more-efficient-string-construction
add_definitions( -DQT_USE_QSTRINGBUILDER )
				

# ------------------------------------------------------------------------------
# OpenGL
# ------------------------------------------------------------------------------
if ( UNIX )
	set(OpenGL_GL_PREFERENCE GLVND)
endif()

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

