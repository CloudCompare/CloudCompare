# ------------------------------------------------------------------------------
# Qt
# ------------------------------------------------------------------------------

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

#hack: we don't want to include Qt debug libs!
#foreach (qt_lib ${QT_LIBRARIES})
#	string( REPLACE d4 4 qt_lib_corrected ${qt_lib})
#	list(APPEND QT_LIBRARIES_NEW ${qt_lib_corrected})
#endforeach()
#set(QT_LIBRARIES ${QT_LIBRARIES_NEW})

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
