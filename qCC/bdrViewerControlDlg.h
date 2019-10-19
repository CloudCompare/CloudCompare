#ifndef BDR_VIEWERCONTROL_DLG_HEADER
#define BDR_VIEWERCONTROL_DLG_HEADER

#include "ui_bdrViewerControlDlg.h"

class ccGLWindow;

namespace Ui
{
	class bdrViewerControlDlg;
}

class bdrViewerControlDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrViewerControlDlg(QWidget* parent = 0, ccGLWindow* win = 0);
	~bdrViewerControlDlg() {}

private:
	Ui::bdrViewerControlDlg	*m_UI;
	ccGLWindow* m_win;

protected slots:

	void doActionZoomGlobal();
	void doActionZoomFit();
	void doActionZoomSelected();
	void doActionViewInfo();
	void doActionRefresh();

};

#endif
