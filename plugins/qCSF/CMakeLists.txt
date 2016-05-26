cmake_minimum_required(VERSION 2.8)

option( INSTALL_QCSF_PLUGIN "Check to install qCSF plugin" OFF )

if (INSTALL_QCSF_PLUGIN)

include (../../CC/CGALSupport.cmake)

#CloudCompare 'CSF' plugin
project( QCSF_PLUGIN )

include( ../CMakePluginTpl.cmake )

include_directories( ${QCSF_PLUGIN_SOURCE_DIR}/csf_h )

target_link_libraries( ${PROJECT_NAME} ${CGAL_LIBRARIES} )

endif()
