#include "bdrLasTilesDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrLasTilesDlg::bdrLasTilesDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrLasTilesDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(AcceptAndExit()));
	connect(m_UI->previewToolButton, &QAbstractButton::clicked, this, &bdrLasTilesDlg::doActioinPreview);
}

void bdrLasTilesDlg::AcceptAndExit()
{

}

void bdrLasTilesDlg::doActioinPreview()
{
}
