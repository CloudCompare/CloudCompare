#ifndef BDR_VIEWERCONTROL_DLG_HEADER
#define BDR_VIEWERCONTROL_DLG_HEADER

#include "ui_bdrViewerControlDlg.h"

namespace Ui
{
	class bdrViewerControlDlg;
}

class bdrViewerControlDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrViewerControlDlg(QWidget* parent = 0);
	~bdrViewerControlDlg() {}

private:
	Ui::bdrViewerControlDlg	*m_UI;

protected slots:



};

#endif
