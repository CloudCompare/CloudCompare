set(header_list
   ${header_list}
   ${CMAKE_CURRENT_SOURCE_DIR}/ccPluginInfoDlg.h
   ${CMAKE_CURRENT_SOURCE_DIR}/ccPluginUIManager.h
   PARENT_SCOPE
)

set(source_list
   ${source_list}
   ${CMAKE_CURRENT_SOURCE_DIR}/ccPluginInfoDlg.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/ccPluginUIManager.cpp
   PARENT_SCOPE
)

set( ui_list
	${ui_list}
	${CMAKE_CURRENT_SOURCE_DIR}/ui/ccPluginInfoDlg.ui
	PARENT_SCOPE
)

set( qrc_list
	${qrc_list}
	${CMAKE_CURRENT_SOURCE_DIR}/pluginManager.qrc
	PARENT_SCOPE
)