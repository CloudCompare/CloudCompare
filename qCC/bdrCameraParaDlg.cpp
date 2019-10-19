#include "bdrCameraParaDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>
#include <QMessageBox>

#include "BlockDBaseIO.h"

bdrCameraParaDlg::bdrCameraParaDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrCameraParaDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));

	//! install the database

}

void bdrCameraParaDlg::setCameraInfo(BlockDB::blkCameraInfo info)
{
	m_UI->cameraNameLineEdit->setText(QString::fromLocal8Bit(info.sName));

	m_UI->pixelSizeDoubleSpinBox->setValue(info.pixelSize);

	m_UI->focalDoubleSpinBox->setValue(info.f);
	m_UI->widthSpinBox->setValue(info.width);
	m_UI->heightSpinBox->setValue(info.height);
	m_UI->pxDoubleSpinBox->setValue(info.x0);
	m_UI->pyDoubleSpinBox->setValue(info.y0);
	
	m_UI->bias0DoubleSpinBox->setValue(info.cameraBias[0]);
	m_UI->bias1DoubleSpinBox->setValue(info.cameraBias[1]);
	m_UI->bias2DoubleSpinBox->setValue(info.cameraBias[2]);
	m_UI->bias3DoubleSpinBox->setValue(info.cameraBias[3]);
	m_UI->bias4DoubleSpinBox->setValue(info.cameraBias[4]);
	m_UI->bias5DoubleSpinBox->setValue(info.cameraBias[5]);

	m_UI->dist0DoubleSpinBox->setValue(info.distortionPar[0]);
	m_UI->dist1DoubleSpinBox->setValue(info.distortionPar[1]);
	m_UI->dist2DoubleSpinBox->setValue(info.distortionPar[2]);
	m_UI->dist3DoubleSpinBox->setValue(info.distortionPar[3]);
	m_UI->dist4DoubleSpinBox->setValue(info.distortionPar[4]);
	m_UI->dist5DoubleSpinBox->setValue(info.distortionPar[5]);
	m_UI->dist6DoubleSpinBox->setValue(info.distortionPar[6]);
	m_UI->dist7DoubleSpinBox->setValue(info.distortionPar[7]);
}

void bdrCameraParaDlg::saveSettings()
{
	if (m_UI->cameraNameLineEdit->text().isEmpty()) {
		QMessageBox::critical(this, "Error!", QString::fromLocal8Bit("请设置相机名"));
		return;
	}
	for (size_t i = 0; i < m_camData.size(); i++) {
		if (QString::fromLocal8Bit(m_camData[i].sName) == m_UI->cameraNameLineEdit->text()) {
			QMessageBox::critical(this, "Error!", QString::fromLocal8Bit("相机名重复"));
			return;
		}
	}
	accept();
}

BlockDB::blkCameraInfo bdrCameraParaDlg::getCameraInfo()
{
	BlockDB::blkCameraInfo info;
	strcpy(info.sName, m_UI->cameraNameLineEdit->text().toLocal8Bit());
	
	info.pixelSize = m_UI->pixelSizeDoubleSpinBox->value();
	info.f=m_UI->focalDoubleSpinBox->value();
	info.width = m_UI->widthSpinBox->value();
	info.height = m_UI->heightSpinBox->value();
	info.x0 = m_UI->pxDoubleSpinBox->value();
	info.y0 = m_UI->pyDoubleSpinBox->value();

	info.cameraBias[0] = m_UI->bias0DoubleSpinBox->value();
	info.cameraBias[1] = m_UI->bias1DoubleSpinBox->value();
	info.cameraBias[2] = m_UI->bias2DoubleSpinBox->value();
	info.cameraBias[3] = m_UI->bias3DoubleSpinBox->value();
	info.cameraBias[4] = m_UI->bias4DoubleSpinBox->value();
	info.cameraBias[5] = m_UI->bias5DoubleSpinBox->value();

	info.distortionPar[0] = m_UI->dist0DoubleSpinBox->value();
	info.distortionPar[1] = m_UI->dist1DoubleSpinBox->value();
	info.distortionPar[2] = m_UI->dist2DoubleSpinBox->value();
	info.distortionPar[3] = m_UI->dist3DoubleSpinBox->value();
	info.distortionPar[4] = m_UI->dist4DoubleSpinBox->value();
	info.distortionPar[5] = m_UI->dist5DoubleSpinBox->value();
	info.distortionPar[6] = m_UI->dist6DoubleSpinBox->value();
	info.distortionPar[7] = m_UI->dist7DoubleSpinBox->value();

	return info;
}
