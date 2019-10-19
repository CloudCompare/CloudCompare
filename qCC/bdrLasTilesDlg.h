#ifndef BDR_LASTILES_DLG_HEADER
#define BDR_LASTILES_DLG_HEADER

#include "ui_bdrLasTilesDlg.h"

namespace Ui
{
	class bdrLasTilesDlg;
}

class bdrLasTilesDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrLasTilesDlg(QWidget* parent = 0);
	~bdrLasTilesDlg() {}

private:
	Ui::bdrLasTilesDlg	*m_UI;

protected slots:

	void AcceptAndExit();

	void doActioinPreview();
};

#endif
