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

find_package( OpenMP QUIET )
if( OpenMP_CXX_FOUND )
    message( STATUS "OpenMP found: compiling CloudCompare with OpenMP support" )
else()
    message( STATUS "OpenMP not found..." )
endif()
