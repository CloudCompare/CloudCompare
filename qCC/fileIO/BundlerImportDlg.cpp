//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QToolButton>
#include <QPushButton>
#include <QFileDialog>
#include <QSettings>

#include <stdio.h>
#include <assert.h>

#include "BundlerImportDlg.h"

BundlerImportDlg::BundlerImportDlg(QWidget* parent)
	: QDialog(parent)
{
    setupUi(this);

	initFromPersistentSettings();

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(acceptAndSaveSettings()));
    connect(browseImageListFileToolButton, SIGNAL(clicked()), this, SLOT(browseImageListFilename()));
    connect(browseAltKeypointsFileToolButton, SIGNAL(clicked()), this, SLOT(browseAltKeypointsFilename	()));
}

BundlerImportDlg::~BundlerImportDlg()
{
}

void BundlerImportDlg::initFromPersistentSettings()
{
    QSettings settings;
    settings.beginGroup("BundlerImport");

	//read parameters
    double scaleFactor		= settings.value("scaleFactor", imageScaleDoubleSpinBox->value()).toDouble();
	bool orthoRectifyAsCloud= settings.value("orthoRectifyAsClouds", orthoRectifyAsCloudCheckBox->isChecked()).toBool();
	bool orthoRectifyAsImage= settings.value("orthoRectifyAsImages", orthoRectifyAsImageCheckBox->isChecked()).toBool();
	bool undistortImages	= settings.value("undistortImages", undistortImagesCheckBox->isChecked()).toBool();
	bool generateColoredDTM	= settings.value("generateColoredDTM", generateColoredDTMCheckBox->isChecked()).toBool();
	bool keepImagesInMemory	= settings.value("keepImagesInMemory", keepImagesInMemoryCheckBox->isChecked()).toBool();
	bool importImages		= settings.value("importImages", imagesGroupBox->isChecked()).toBool();
	bool useAltKeypoints	= /*settings.value("useAltKeypoints", altKeypointsGroupBox->isChecked()).toBool()*/false; //DGM: if we don't handle the filename, it's too dangerous
	bool importKeypoints	= settings.value("importKeypoints", importKeypointsCheckBox->isChecked()).toBool();
	int dtmVerticesCount	= settings.value("dtmVerticesCount", dtmVerticesSpinBox->value()).toInt();

	//apply parameters
	imageScaleDoubleSpinBox->setValue(scaleFactor);
	orthoRectifyAsCloudCheckBox->setChecked(orthoRectifyAsCloud);
	orthoRectifyAsImageCheckBox->setChecked(orthoRectifyAsImage);
	undistortImagesCheckBox->setChecked(undistortImages);
	generateColoredDTMCheckBox->setChecked(generateColoredDTM);
	imagesGroupBox->setChecked(importImages);
	altKeypointsGroupBox->setChecked(useAltKeypoints);
	importKeypointsCheckBox->setChecked(importKeypoints);
	dtmVerticesSpinBox->setValue(dtmVerticesCount);
	keepImagesInMemoryCheckBox->setChecked(keepImagesInMemory);

	settings.endGroup();
}

void BundlerImportDlg::acceptAndSaveSettings()
{
    QSettings settings;
    settings.beginGroup("BundlerImport");

	//write parameters
    settings.setValue("scaleFactor", imageScaleDoubleSpinBox->value());
	settings.setValue("orthoRectifyAsClouds", orthoRectifyAsCloudCheckBox->isChecked());
	settings.setValue("orthoRectifyAsImages", orthoRectifyAsImageCheckBox->isChecked());
	settings.setValue("undistortImages", undistortImagesCheckBox->isChecked());
	settings.setValue("generateColoredDTM", generateColoredDTMCheckBox->isChecked());
	settings.setValue("keepImagesInMemory", keepImagesInMemoryCheckBox->isChecked());
	settings.setValue("importImages", imagesGroupBox->isChecked());
	settings.setValue("useAltKeypoints", altKeypointsGroupBox->isChecked());
	settings.setValue("importKeypoints", importKeypointsCheckBox->isChecked());
	settings.setValue("dtmVerticesCount", dtmVerticesSpinBox->value());

    settings.endGroup();
}

bool BundlerImportDlg::useAlternativeKeypoints() const
{
	return altKeypointsGroupBox->isChecked();
}

bool BundlerImportDlg::importKeypoints() const
{
	return importKeypointsCheckBox->isEnabled() && importKeypointsCheckBox->isChecked();
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
	return imagesGroupBox->isEnabled() && generateColoredDTMCheckBox->isChecked();
}

bool BundlerImportDlg::keepImagesInMemory() const
{
	return imagesGroupBox->isEnabled() && keepImagesInMemoryCheckBox->isChecked();
}

void BundlerImportDlg::setKeypointsCount(unsigned count)
{
	keyPointsCountLabel->setText(QString::number(count));
	importKeypointsCheckBox->setEnabled(count>0);
	if (count==0) //can't ortho-rectify without keypoints!
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
	imagesGroupBox->setEnabled(count>0);
}

void BundlerImportDlg::setVer(unsigned majorVer, unsigned minorVer)
{
	majorVerLabel->setText(QString("v%1").arg(majorVer));
	minorVerLabel->setText(QString(".%1").arg(minorVer));
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
	QString imageListFilename = QFileDialog::getOpenFileName(this,
															"Open image list file",
															imageListFilePathLineEdit->text(),
															"Image list (*.txt)");

	if (!imageListFilename.isEmpty())
		imageListFilePathLineEdit->setText(imageListFilename);
}

void BundlerImportDlg::browseAltKeypointsFilename()
{
	QString altKeypointsFilename = QFileDialog::getOpenFileName(this,
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
