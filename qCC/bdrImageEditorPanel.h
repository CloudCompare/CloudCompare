#ifndef BDR_IMAGE_EDITOR_PANEL_HEADER
#define BDR_IMAGE_EDITOR_PANEL_HEADER

#include "ui_bdrImageEditorPanel.h"
class MainWindow;
class bdr2Point5DimEditor;
class ccDBRoot;
class ccHObject;

//! Dialog for qRansacSD plugin
class bdrImageEditorPanel : public QDialog, public Ui::bdrImageEditorPanelDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrImageEditorPanel(bdr2Point5DimEditor* img, ccDBRoot* root, QWidget* parent = 0);


protected slots:
	void ZoomFit();
	void toogleImageList();
	void changeSelection();
	void displayImage();
	void selectImage();
signals:
	void imageDisplayed();
public:
	void setItems(std::vector<ccHObject*> items, int defaultSelectedIndex);
private:
	bdr2Point5DimEditor* m_pbdrImshow;
	ccDBRoot* m_root;
	
	int m_image_display_height;
};

#endif
