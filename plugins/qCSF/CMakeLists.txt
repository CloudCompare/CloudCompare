cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCSF_PLUGIN "Check to install qCSF plugin" OFF )

if (INSTALL_QCSF_PLUGIN)

option( QCSF_CGAL_SUPPORT "Check to compile qCSF with CGAL support (warning: slower)" )
if (QCSF_CGAL_SUPPORT)
include (../../CC/CGALSupport.cmake)
endif()

#CloudCompare 'CSF' plugin
project( QCSF_PLUGIN )

include( ../CMakePluginTpl.cmake )

if (QCSF_CGAL_SUPPORT)
target_link_libraries( ${PROJECT_NAME} ${CGAL_LIBRARIES} )
set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS WITH_CGAL )
endif()

endif()
