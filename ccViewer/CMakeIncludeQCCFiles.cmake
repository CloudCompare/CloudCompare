# Include specific capabilities from qCC

list( APPEND header_list
	../qCC/ccCameraParamEditDlg.h
	../qCC/ccDisplayOptionsDlg.h
	../qCC/ccOptions.h
	../qCC/ccOverlayDialog.h
	../qCC/ccPickingHub.h
	../qCC/ccPickOneElementDlg.h
	../qCC/ccStereoModeDlg.h
	../qCC/pluginManager/ccPluginManager.h
)

list( APPEND source_list
	../qCC/ccCameraParamEditDlg.cpp
	../qCC/ccDisplayOptionsDlg.cpp
	../qCC/ccOptions.cpp
	../qCC/ccOverlayDialog.cpp
	../qCC/ccPickingHub.cpp
	../qCC/ccPickOneElementDlg.cpp
	../qCC/ccStereoModeDlg.cpp
	../qCC/pluginManager/ccPluginManager.cpp
)

list( APPEND ui_list
	../qCC/ui_templates/cameraParamDlg.ui
	../qCC/ui_templates/displayOptionsDlg.ui
	../qCC/ui_templates/pickOneElementDlg.ui
	../qCC/ui_templates/stereoModeDlg.ui
)
