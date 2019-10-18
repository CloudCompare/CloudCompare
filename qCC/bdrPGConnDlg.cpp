#include "bdrPGConnDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrPGConnDlg::bdrPGConnDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrPGConnDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}

void bdrPGConnDlg::saveSettings()
{

}