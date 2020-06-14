//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "BundlerImportDlg.h"

//Qt
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QToolButton>
#include <QPushButton>
#include <QFileDialog>
#include <QSettings>
#include <QMessageBox>

//system
#include <stdio.h>
#include <assert.h>

//qCC_db
#include <ccGLMatrix.h>

BundlerImportDlg::BundlerImportDlg(QWidget* parent)
	: QDialog(parent)
{
	setupUi(this);

	applyTransfoMatrixTextEdit->setVisible(false);

	initFromPersistentSettings();

	connect(buttonBox, &QDialogButtonBox::accepted, this, &BundlerImportDlg::acceptAndSaveSettings);
	connect(browseImageListFileToolButton, &QAbstractButton::clicked, this, &BundlerImportDlg::browseImageListFilename);
	connect(browseAltKeypointsFileToolButton, &QAbstractButton::clicked, this, &BundlerImportDlg::browseAltKeypointsFilename);
}

void BundlerImportDlg::initFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("BundlerImport");

	//read parameters
	double scaleFactor			= settings.value("scaleFactor", imageScaleDoubleSpinBox->value()).toDouble();
	bool orthoRectifyAsCloud	= settings.value("orthoRectifyAsClouds", orthoRectifyAsCloudCheckBox->isChecked()).toBool();
	bool orthoRectifyAsImage	= settings.value("orthoRectifyAsImages", orthoRectifyAsImageCheckBox->isChecked()).toBool();
	bool undistortImages		= settings.value("undistortImages", undistortImagesCheckBox->isChecked()).toBool();
	bool generateColoredDTM		= settings.value("generateColoredDTM", generateColoredDTMGroupBox->isChecked()).toBool();
	bool keepImagesInMemory		= settings.value("keepImagesInMemory", keepImagesInMemoryCheckBox->isChecked()).toBool();
	bool importImages			= settings.value("importImages", imagesGroupBox->isChecked()).toBool();
	bool useAltKeypoints		= /*settings.value("useAltKeypoints", altKeypointsCheckBox->isChecked()).toBool()*/false; //DGM: if we don't handle the filename, it's too dangerous
	bool importKeypoints		= settings.value("importKeypoints", importKeypointsGroupBox->isChecked()).toBool();
	int dtmVerticesCount		= settings.value("dtmVerticesCount", dtmVerticesSpinBox->value()).toInt();
	int orthoRectMethod			= settings.value("orthoRectMethod", orthoRectMethodComboBox->currentIndex()).toInt();

	//apply parameters
	imageScaleDoubleSpinBox->setValue(scaleFactor);
	orthoRectifyAsCloudCheckBox->setChecked(orthoRectifyAsCloud);
	orthoRectifyAsImageCheckBox->setChecked(orthoRectifyAsImage);
	undistortImagesCheckBox->setChecked(undistortImages);
	generateColoredDTMGroupBox->setChecked(generateColoredDTM);
	imagesGroupBox->setChecked(importImages);
	altKeypointsCheckBox->setChecked(useAltKeypoints);
	importKeypointsGroupBox->setChecked(importKeypoints);
	dtmVerticesSpinBox->setValue(dtmVerticesCount);
	keepImagesInMemoryCheckBox->setChecked(keepImagesInMemory);
	orthoRectMethodComboBox->setCurrentIndex(orthoRectMethod);

	settings.endGroup();
}

void BundlerImportDlg::acceptAndSaveSettings()
{
	//check matrix validity
	if (customVertAxisRadioButton->isChecked())
	{
		bool success;
		ccGLMatrixd::FromString(applyTransfoMatrixTextEdit->toPlainText(),success);
		if (!success)
		{
			QMessageBox::critical(this,"Invalid matrix","Invalid input 4x4 matrix!");
			return;
		}
	}

	QSettings settings;
	settings.beginGroup("BundlerImport");

	//write parameters
	settings.setValue("scaleFactor", imageScaleDoubleSpinBox->value());
	settings.setValue("orthoRectifyAsClouds", orthoRectifyAsCloudCheckBox->isChecked());
	settings.setValue("orthoRectifyAsImages", orthoRectifyAsImageCheckBox->isChecked());
	settings.setValue("undistortImages", undistortImagesCheckBox->isChecked());
	settings.setValue("generateColoredDTM", generateColoredDTMGroupBox->isChecked());
	settings.setValue("keepImagesInMemory", keepImagesInMemoryCheckBox->isChecked());
	settings.setValue("importImages", imagesGroupBox->isChecked());
	settings.setValue("useAltKeypoints", altKeypointsCheckBox->isChecked());
	settings.setValue("importKeypoints", importKeypointsGroupBox->isChecked());
	settings.setValue("dtmVerticesCount", dtmVerticesSpinBox->value());
	settings.setValue("orthoRectMethod", orthoRectMethodComboBox->currentIndex());

	settings.endGroup();

	accept();
}

bool BundlerImportDlg::useAlternativeKeypoints() const
{
	return altKeypointsCheckBox->isChecked();
}

bool BundlerImportDlg::importKeypoints() const
{
	return importKeypointsGroupBox->isEnabled() && importKeypointsGroupBox->isChecked();
}

bool BundlerImportDlg::importImages() const
{
	return imagesGroupBox->isEnabled() && imagesGroupBox->isChecked();
}

bool BundlerImportDlg::undistortImages() const
{
	return imagesGroupBox->isEnabled() && undistortImagesCheckBox->isEnabled() && undistortImagesCheckBox->isChecked();
}

bool BundlerImportDlg::orthoRectifyImagesAsClouds() const
{
	return imagesGroupBox->isEnabled() && orthoRectifyAsCloudCheckBox->isChecked();
}

bool BundlerImportDlg::orthoRectifyImagesAsImages() const
{
	return imagesGroupBox->isEnabled() && orthoRectifyAsImageCheckBox->isChecked();
}

bool BundlerImportDlg::generateColoredDTM() const
{
	return imagesGroupBox->isEnabled() && generateColoredDTMGroupBox->isChecked();
}

bool BundlerImportDlg::keepImagesInMemory() const
{
	return imagesGroupBox->isEnabled() && keepImagesInMemoryCheckBox->isChecked();
}

BundlerImportDlg::OrthoRectMethod BundlerImportDlg::getOrthorectificationMethod() const
{
	switch (orthoRectMethodComboBox->currentIndex())
	{
	case 0:
		return OPTIMIZED;
	case 1:
		return DIRECT_UNDISTORTED;
	case 2:
		return DIRECT;
	default:
		assert(false);
		break;
	}

	return OPTIMIZED;
}

void BundlerImportDlg::setKeypointsCount(unsigned count)
{
	keyPointsCountLabel->setText(QString::number(count));
	importKeypointsGroupBox->setEnabled(count != 0);
	if (count == 0) //can't ortho-rectify without keypoints!
	{
		orthoRectifyAsImageCheckBox->setChecked(false);
		orthoRectifyAsImageCheckBox->setEnabled(false);
		orthoRectifyAsCloudCheckBox->setChecked(false);
		orthoRectifyAsCloudCheckBox->setEnabled(false);
	}
}

void BundlerImportDlg::setCamerasCount(unsigned count)
{
	cameraCountLabel->setText(QString::number(count));
	imagesGroupBox->setEnabled(count != 0);
}

void BundlerImportDlg::setVer(unsigned majorVer, unsigned minorVer)
{
	versionLabel->setText(QString("v%1.%2").arg(majorVer).arg(minorVer));
}

void BundlerImportDlg::setImageListFilename(const QString& filename)
{
	imageListFilePathLineEdit->setText(filename);
}

QString BundlerImportDlg::getImageListFilename() const
{
	return imageListFilePathLineEdit->text();
}

void BundlerImportDlg::setAltKeypointsFilename(const QString& filename)
{
	altKeypointsFilePathLineEdit->setText(filename);
}

QString BundlerImportDlg::getAltKeypointsFilename() const
{
	return altKeypointsFilePathLineEdit->text();
}

void BundlerImportDlg::browseImageListFilename()
{
	QString imageListFilename =
		QFileDialog::getOpenFileName(	this,
										"Open image list file",
										imageListFilePathLineEdit->text(),
										"Image list (*.txt)");

	if (!imageListFilename.isEmpty())
		imageListFilePathLineEdit->setText(imageListFilename);
}

void BundlerImportDlg::browseAltKeypointsFilename()
{
	QString altKeypointsFilename =
		QFileDialog::getOpenFileName(	this,
										"Open alternative keypoints file",
										altKeypointsFilePathLineEdit->text(),
										"Cloud/mesh (*.*)");

	if (!altKeypointsFilename.isEmpty())
		altKeypointsFilePathLineEdit->setText(altKeypointsFilename);
}

double BundlerImportDlg::getScaleFactor() const
{
	return imageScaleDoubleSpinBox->value();
}

unsigned BundlerImportDlg::getDTMVerticesCount() const
{
	return dtmVerticesSpinBox->value();
}

bool BundlerImportDlg::getOptionalTransfoMatrix(ccGLMatrix& mat)
{
	if (xVertAxisRadioButton->isChecked())
	{
		//rotation around Y
		mat.toZero();
		mat.data()[2]  = -1.0f;
		mat.data()[5]  =  1.0f;
		mat.data()[8]  =  1.0f;
		mat.data()[15] =  1.0f;
		return true;
	}
	else if (yVertAxisRadioButton->isChecked())
	{
		//rotation around X
		mat.toZero();
		mat.data()[0]  =  1.0f;
		mat.data()[6]  = -1.0f;
		mat.data()[9]  = -1.0f;
		mat.data()[15] =  1.0f;
		return true;
	}
	else if (customVertAxisRadioButton->isChecked())
	{
		bool success = false;
		mat = ccGLMatrix::FromString(applyTransfoMatrixTextEdit->toPlainText(),success);
		return success;
	}

	return false;

}
