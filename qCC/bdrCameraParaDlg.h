#ifndef BDR_CAMERAPARA_DLG_HEADER
#define BDR_CAMERAPARA_DLG_HEADER

#include "ui_bdrCameraParaDlg.h"

namespace Ui
{
	class bdrCameraParaDlg;
}

class bdrCameraParaDlg : public QDialog
{
	Q_OBJECT

public:

	explicit bdrCameraParaDlg(QWidget* parent = 0);
	~bdrCameraParaDlg() {}

private:
	Ui::bdrCameraParaDlg	*m_UI;

protected slots:

	void saveSettings();


};

#endif
