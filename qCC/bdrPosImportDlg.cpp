#include "bdrPosImportDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrPosImportDlg::bdrPosImportDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrPosImportDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(AcceptAndExit()));
}

void bdrPosImportDlg::AcceptAndExit()
{

}

QString bdrPosImportDlg::getPosPath()
{
	return m_UI->pathLineEdit->text();
}

std::vector<BlockDB::blkImageInfo> bdrPosImportDlg::getImagePosInfo()
{
	std::vector<BlockDB::blkImageInfo> cam;
	BlockDB::readPosFile(getPosPath().toLocal8Bit(), cam);
	return cam;
}
