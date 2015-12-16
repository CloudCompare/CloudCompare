# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------
option( USE_QT5 "Check to use Qt5 instead of Qt4" OFF )
## we will use cmake automoc feature
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if ( USE_QT5 )

	cmake_minimum_required(VERSION 2.8.8)

	# go straight to find qt5
    # find_package(Qt5 COMPONENTS OpenGL Widgets Core Gui PrintSupport Concurrent REQUIRED)
	find_package(Qt5Widgets)
	find_package(Qt5Core)
	find_package(Qt5Gui)
	find_package(Qt5PrintSupport)
	find_package(Qt5Concurrent)
        find_package(Qt5OpenGL)

	# in the case no Qt5Config.cmake file could be found, cmake will explicitly ask the user for the QT5_DIR containing it!
	# thus no need to keep additional variables and checks

	if ( MSVC )
		# Where to find OpenGL libraries
		set(WINDOWS_OPENGL_LIBS "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64" CACHE PATH "WindowsSDK libraries" )
		list( APPEND CMAKE_PREFIX_PATH ${WINDOWS_OPENGL_LIBS} )
	endif()

	get_target_property(QT5_LIB_LOCATION Qt5::Core LOCATION_${CMAKE_BUILD_TYPE})
	get_filename_component(QT_BINARY_DIR ${QT5_LIB_LOCATION} DIRECTORY)
		
	set(QT5_ROOT_PATH ${QT_BINARY_DIR}/../)

    include_directories(${Qt5OpenGL_INCLUDE_DIRS}
                        ${Qt5Widgets_INCLUDE_DIRS}
                        ${Qt5Core_INCLUDE_DIRS}
                        ${Qt5Gui_INCLUDE_DIRS}
                        ${Qt5Concurrent_INCLUDE_DIRS}
                        ${Qt5PrintSupport_INCLUDE_DIRS})
else() # using qt4

	set( DESIRED_QT_VERSION 4 )
	set (QT_BINARY_DIR "") #to force CMake to update QT_BINARY_DIR!
	if ( MSVC )
		#We need QtMain to use 'WIN32' mode (/subsystem:windows) with MSVC
		find_package( Qt4 ${QT_VERSION} COMPONENTS QtMain QtCore QtGui QtOpenGL REQUIRED )
	else()
		find_package( Qt4 ${QT_VERSION} COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
	endif()
	if( NOT QT_FOUND )
		message( SEND_ERROR "Qt required, but not found with 'find_package()'" )
	else()
		include( ${QT_USE_FILE} )
	endif()
	
        include_directories(${QT_INCLUDE_DIR})
endif()

# ------------------------------------------------------------------------------
# OpenGL
# ------------------------------------------------------------------------------

find_package( OpenGL REQUIRED )
if( NOT OPENGL_FOUND )
    message( SEND_ERROR "OpenGL required, but not found with 'find_package()'" )
endif()

include_directories(${OpenGL_INCLUDE_DIR})
# ------------------------------------------------------------------------------
# CUDA
# ------------------------------------------------------------------------------
#if( USE_CUDA )
#    find_package( CUDA REQUIRED )
#    if( NOT CUDA_FOUND )
#        message( SEND_ERROR "CUDA required, but not found with 'find_package()'" )
#    endif()
#endif()

# ------------------------------------------------------------------------------
# OpenMP (only on UNIX for now)
# ------------------------------------------------------------------------------
if (UNIX)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        message("OpenMP found, I am gonna use it")
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread" )
    endif()
endif()

# ------------------------------------------------------------------------------
# Some macros for easily passing from qt4 to qt5 when we will be ready
# ------------------------------------------------------------------------------
macro(qt_wrap_ui)
    if(USE_QT5)
        qt5_wrap_ui(${ARGN})
    else()
        qt4_wrap_ui(${ARGN})
    endif()
endmacro()


macro(qt_add_resources)
    if(USE_QT5)
        qt5_add_resources(${ARGN})
    else()
        qt4_add_resources(${ARGN})
    endif()
endmacro()
