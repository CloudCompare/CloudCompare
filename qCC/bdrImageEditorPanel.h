#ifndef BDR_IMAGE_EDITOR_PANEL_HEADER
#define BDR_IMAGE_EDITOR_PANEL_HEADER

#include "ui_bdrImageEditorPanel.h"
class MainWindow;
class bdr2Point5DimEditor;
class ccDBRoot;
class ccHObject;
class ccBBox;
class ccCameraSensor;
class bdrTraceFootprint;
#include "ccBBox.h"

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
	void previous();
	void next();
	void toogleDisplayAll();
	void startEditor();
	void stopEditor(bool);
	
signals:
	void imageDisplayed();
public:
	void clearAll();
	void setItems(std::vector<ccHObject*> items, int defaultSelectedIndex);
	void setItems(std::vector<ccCameraSensor*> items, int defaultSelectedIndex);
	void display(bool display_all);
	double getBoxScale();
	ccBBox getObjBox(); 
	void setObjBox(ccBBox box); 
	bool isObjChecked();
	
private:
	bdr2Point5DimEditor* m_pbdrImshow;
	bdrTraceFootprint* m_pbdrTraceFP;
	ccDBRoot* m_root;
	ccBBox m_obj_box;
	int m_image_display_height;
};

#endif
