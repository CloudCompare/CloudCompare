# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------

option( USE_QT5 "Check to use Qt5 instead of Qt4" OFF )
if ( USE_QT5 )

	cmake_minimum_required(VERSION 2.8.8)

	set( QT5_ROOT_PATH CACHE PATH "Qt5 root directory (i.e. where the 'bin' folder lies)" )
	if ( QT5_ROOT_PATH )
	
		list( APPEND CMAKE_PREFIX_PATH ${QT5_ROOT_PATH} )
	
		#see http://www.kdab.com/using-cmake-with-qt-5/
		# Find includes in corresponding build directories
		set(CMAKE_INCLUDE_CURRENT_DIR ON)
		# Instruct CMake to run moc automatically when needed.
		set(CMAKE_AUTOMOC ON)
		
		if ( MSVC )
			# Where to find opengl libraries
			set(WINDOWS_OPENGL_LIBS "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64" CACHE PATH "WindowsSDK libraries" )
			list( APPEND CMAKE_PREFIX_PATH ${WINDOWS_OPENGL_LIBS} )
		endif()

		# Find the Qt5 libraries
		#set( DESIRED_QT_VERSION 5 )
		find_package(Qt5OpenGL REQUIRED)
		find_package(Qt5Widgets REQUIRED)
		find_package(Qt5Core REQUIRED)
		find_package(Qt5Gui REQUIRED)
		find_package(Qt5Concurrent REQUIRED)
		
	else()
		message(SEND_ERROR "Please specify the Qt5 installation root directory")
	endif()

	list( APPEND EXTERNAL_LIBS_INCLUDE_DIR ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Concurrent_INCLUDE_DIRS} )
	#list( APPEND EXTERNAL_LIBS_LIBRARIES ${Qt5OpenGL_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Concurrent_LIBRARIES} )

	#for executables only!
	#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5OpenGL_EXECUTABLE_COMPILE_FLAGS}")
	#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
	#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS}")
	#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS}")
	
else()

	set( DESIRED_QT_VERSION 4 )
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
	
	list( APPEND EXTERNAL_LIBS_INCLUDE_DIR ${QT_INCLUDE_DIR} )
	list( APPEND EXTERNAL_LIBS_LIBRARIES ${QT_LIBRARIES} )

endif()

# ------------------------------------------------------------------------------
# OpenGL
# ------------------------------------------------------------------------------

find_package( OpenGL REQUIRED )
if( NOT OPENGL_FOUND )
    message( SEND_ERROR "OpenGL required, but not found with 'find_package()'" )
endif()

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
# Global variables
# ------------------------------------------------------------------------------

list( APPEND EXTERNAL_LIBS_INCLUDE_DIR ${QT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR}  )
list( APPEND EXTERNAL_LIBS_LIBRARIES ${QT_LIBRARIES} ${OPENGL_LIBRARIES} )
