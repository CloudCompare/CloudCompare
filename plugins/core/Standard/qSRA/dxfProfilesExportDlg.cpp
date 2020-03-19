//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#include "ccFileUtils.h"

#include "dxfProfilesExportDlg.h"

//Qt
#include <QFileDialog>
#include <QSettings>
#include <QFileInfo>

//System
#include <assert.h>

DxfProfilesExportDlg::DxfProfilesExportDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::DxfProfilesExportDlg()
{
	setupUi(this);

	connect(vertBrowseToolButton, &QAbstractButton::clicked, this, &DxfProfilesExportDlg::browseVertFile);
	connect(horizBrowseToolButton, &QAbstractButton::clicked, this, &DxfProfilesExportDlg::browseHorizFile);
	connect(buttonBox, &QDialogButtonBox::accepted, this, &DxfProfilesExportDlg::acceptAndSaveSettings);

	initFromPersistentSettings();
}

void DxfProfilesExportDlg::initFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("DxfProfilesExportDialog");

	const QString	defaultVertProfile( ccFileUtils::defaultDocPath() + "/vert_profiles.dxf" );
	const QString	defaultHorizProfile( ccFileUtils::defaultDocPath() + "/horiz_profiles.dxf" );
	
	//read parameters
	bool vertEnabled		= settings.value("vertExportGroup",	true).toBool();
	bool horizEnabled		= settings.value("horizExportGroup", true).toBool();
	QString vertPath		= settings.value("vertExportPath",	defaultVertProfile).toString();
	QString horizPath		= settings.value("horizExportPath",	defaultHorizProfile).toString();
	QString vertTitle		= settings.value("vertTitle",		vertTitleLineEdit->text()).toString();
	QString horizTitle		= settings.value("horizTitle",		horizTitleLineEdit->text()).toString();
	QString theoTitle		= settings.value("legendTheoTitle",	theoNameLineEdit->text()).toString();
	QString realTitle		= settings.value("legendRealTitle",	realNameLineEdit->text()).toString();
	int angularSteps		= settings.value("angularSteps",	angularStepsSpinBox->value()).toInt();
	int heightSteps			= settings.value("heightSteps",		heightStepsSpinBox->value()).toInt();
	double devScale			= settings.value("devScale",		devValuesScaleDoubleSpinBox->value()).toDouble();
	QString scaledDevUnits	= settings.value("scaledDevUnits",	scaledDevUnitsLineEdit->text()).toString();
	int precision			= settings.value("precision",		precisionSpinBox->value()).toInt();
	int magnifyCoef			= settings.value("magnifyCoef",		magnifyCoefSpinBox->value()).toInt();
	
	//apply parameters
	vertProfilesGroupBox->setChecked(vertEnabled);
	horizProfilesGroupBox->setChecked(horizEnabled);
	vertOutputFileLineEdit->setText(vertPath);
	horizOutputFileLineEdit->setText(horizPath);
	vertTitleLineEdit->setText(vertTitle);
	horizTitleLineEdit->setText(horizTitle);
	theoNameLineEdit->setText(theoTitle);
	realNameLineEdit->setText(realTitle);
	angularStepsSpinBox->setValue(angularSteps);
	heightStepsSpinBox->setValue(heightSteps);
	devValuesScaleDoubleSpinBox->setValue(devScale);
	scaledDevUnitsLineEdit->setText(scaledDevUnits);
	precisionSpinBox->setValue(precision);
	magnifyCoefSpinBox->setValue(magnifyCoef);

	settings.endGroup();
}

void DxfProfilesExportDlg::acceptAndSaveSettings()
{
	QSettings settings;
	settings.beginGroup("DxfProfilesExportDialog");

	//write parameters
	settings.setValue("vertExportGroup",	vertProfilesGroupBox->isChecked());
	settings.setValue("horizExportGroup",	horizProfilesGroupBox->isChecked());
	if (vertProfilesGroupBox->isChecked())
		settings.setValue("vertExportPath",		vertOutputFileLineEdit->text());
	if (horizProfilesGroupBox->isChecked())
		settings.setValue("horizExportPath",	horizOutputFileLineEdit->text());
	settings.setValue("vertTitle",			vertTitleLineEdit->text());
	settings.setValue("horizTitle",			horizTitleLineEdit->text());
	settings.setValue("legendTheoTitle",	theoNameLineEdit->text());
	settings.setValue("legendRealTitle",	realNameLineEdit->text());
	settings.setValue("angularSteps",		angularStepsSpinBox->value());
	settings.setValue("heightSteps",		heightStepsSpinBox->value());
	settings.setValue("devScale",			devValuesScaleDoubleSpinBox->value());
	settings.setValue("scaledDevUnits",		scaledDevUnitsLineEdit->text());
	settings.setValue("precision",			precisionSpinBox->value());
	settings.setValue("magnifyCoef",		magnifyCoefSpinBox->value());

	settings.endGroup();
}

void DxfProfilesExportDlg::browseVertFile()
{
	QString filter("Vertical profiles (*.dxf)");

	//open file loading dialog
	QString filename = QFileDialog::getSaveFileName(nullptr,"Select output file",vertOutputFileLineEdit->text(),filter);

	if (filename.isEmpty())
		return;

	vertOutputFileLineEdit->setText(filename);
}

QString DxfProfilesExportDlg::getVertFilename() const
{
	return vertProfilesGroupBox->isChecked() ? vertOutputFileLineEdit->text() : QString();
}

void DxfProfilesExportDlg::browseHorizFile()
{
	QString filter("Horizontal profiles (*.dxf)");

	//open file loading dialog
	QString filename = QFileDialog::getSaveFileName(nullptr,"Select output file",horizOutputFileLineEdit->text(),filter);

	if (filename.isEmpty())
		return;

	horizOutputFileLineEdit->setText(filename);
}

QString DxfProfilesExportDlg::getHorizFilename() const
{
	return horizProfilesGroupBox->isChecked() ? horizOutputFileLineEdit->text() : QString();
}
