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
	connect(m_UI->openFileToolButton, &QAbstractButton::clicked, this, &bdrPosImportDlg::doActionOpenFile);
}

void bdrPosImportDlg::AcceptAndExit()
{

}
#include <QSettings>
#include <QFileInfo>
#include "ccPersistentSettings.h"
#include "ccFileUtils.h"
void bdrPosImportDlg::doActionOpenFile()
{
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	//file choosing dialog
	QString selectedFiles = QFileDialog::getOpenFileName(this,
		tr("Open file(s)"),
		currentPath,
		("All (*.*);;txt(*.txt)"),
		&QString("txt(*.txt)"));
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles).absoluteFilePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();

	m_UI->pathLineEdit->setText(selectedFiles);
}

QString bdrPosImportDlg::getPosPath()
{
	return m_UI->pathLineEdit->text();
}

std::vector<BlockDB::blkImageInfo> bdrPosImportDlg::getImagePosInfo()
{
	std::vector<BlockDB::blkImageInfo> cam;
	BlockDB::loadPosFile(getPosPath().toLocal8Bit(), cam);
	return cam;
}
