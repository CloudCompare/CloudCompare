#include "bdrPosImportDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrPosImportDlg::bdrPosImportDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrPosImportDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}

void bdrPosImportDlg::saveSettings()
{

}