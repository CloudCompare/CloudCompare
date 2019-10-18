#include "bdrCameraParaDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrCameraParaDlg::bdrCameraParaDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrCameraParaDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}

void bdrCameraParaDlg::saveSettings()
{

}