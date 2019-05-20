#ifndef BDR_IMAGE_EDITOR_PANEL_HEADER
#define BDR_IMAGE_EDITOR_PANEL_HEADER

#include "ui_bdrImageEditorPanel.h"
class MainWindow;
class bdr2Point5DimEditor;

//! Dialog for qRansacSD plugin
class bdrImageEditorPanel : public QDialog, public Ui::bdrImageEditorPanelDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrImageEditorPanel(bdr2Point5DimEditor* img, QWidget* parent = 0);


protected slots:
	void ZoomFit();

private:
	bdr2Point5DimEditor* m_pbdrImshow;
};

#endif
