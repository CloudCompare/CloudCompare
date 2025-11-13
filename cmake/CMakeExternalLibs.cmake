# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------

find_package( Qt6 REQUIRED
    COMPONENTS
        Concurrent
        Core
        Gui
        OpenGL
		OpenGLWidgets
        PrintSupport
        Svg
        Widgets
)

qt_standard_project_setup()

# turn on QStringBuilder for more efficient string construction
#	see https://doc.qt.io/qt-6/qstring.html#more-efficient-string-construction
add_definitions( -DQT_USE_QSTRINGBUILDER )


# ------------------------------------------------------------------------------
# OpenGL
# ------------------------------------------------------------------------------

# We could need this for raw openGL calls
# but we access OpenGL functions through Qt's function pointers...
#if ( UNIX and NOT APPLE)
	#set(OpenGL_GL_PREFERENCE GLVND)
#endif()
#find_package(OpenGL REQUIRED)


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
