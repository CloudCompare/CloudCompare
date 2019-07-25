#ifndef BDR_IMAGE_EDITOR_PANEL_HEADER
#define BDR_IMAGE_EDITOR_PANEL_HEADER

#include "ui_bdrImageEditorPanel.h"
class MainWindow;
class bdr2Point5DimEditor;
class ccDBRoot;
class ccHObject;
class ccBBox;
class ccCameraSensor;
class bdrSketcher;
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
	ccHObject * getTraceBlock(QString image_name);
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

	ccBBox getObjViewBox() { return m_objViewBox; }
	void setObjViewBox(ccBBox box) { m_objViewBox = box; }
	CCVector3d getObjViewUpDir() { return m_objViewUpDir.norm2d() < 1e-6 ? CCVector3d(0, 1, 0) : m_objViewUpDir; }
	void setObjViewUpDir(CCVector3d up) { m_objViewUpDir = up; }

	bool isObjChecked();
	void updateCursorPos(const CCVector3d& P, bool b3d);
	bool isLinkToMainView();
private:
	bdr2Point5DimEditor* m_pbdrImshow;
	bdrSketcher* m_pbdrTraceFP;
	ccDBRoot* m_root;
	ccBBox m_objViewBox;
	CCVector3d m_objViewUpDir;
	int m_image_display_height;
};

#endif
