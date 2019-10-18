#ifndef BDR_POSIMPORT_DLG_HEADER
#define BDR_POSIMPORT_DLG_HEADER

#include "ui_bdrPosImportDlg.h"

namespace Ui
{
	class bdrPosImportDlg;
}

class bdrPosImportDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrPosImportDlg(QWidget* parent = 0);
	~bdrPosImportDlg() {}

private:
	Ui::bdrPosImportDlg	*m_UI;

protected slots:

	void saveSettings();


};

#endif
