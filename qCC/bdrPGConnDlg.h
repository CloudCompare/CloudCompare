#ifndef BDR_PGCONN_DLG_HEADER
#define BDR_PGCONN_DLG_HEADER

#include "ui_bdrPGConnDlg.h"

namespace Ui
{
	class bdrPGConnDlg;
}

class bdrPGConnDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrPGConnDlg(QWidget* parent = 0);
	~bdrPGConnDlg() {}

private:
	Ui::bdrPGConnDlg	*m_UI;

protected slots:

	void AcceptAndExit();


	bool doActionTestConnection();

};

#endif
