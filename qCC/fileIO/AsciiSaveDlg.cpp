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

#include "AsciiSaveDlg.h"

//Qt
#include <QSpinBox>
#include <QComboBox>
#include <QSettings>

//system
#include <assert.h>

AsciiSaveDlg::AsciiSaveDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::AsciiSaveDialog()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(acceptAndSaveSettings()));

	initFromPersistentSettings();
}

AsciiSaveDlg::~AsciiSaveDlg()
{
}

bool AsciiSaveDlg::saveColumnsNamesHeader() const
{
	return columnsHeaderCheckBox->isChecked();
}

bool AsciiSaveDlg::savePointCountHeader() const
{
	return pointCountHeaderCheckBox->isChecked();
}

uchar AsciiSaveDlg::getSeparator() const
{
	switch(separatorComboBox->currentIndex())
	{
	case 0:
		return ' ';
	case 1:
		return ';';
	case 2:
		return ',';
	case 3:
		return '\t';
	default:
		assert(false);
	}

	return 0;
}

int AsciiSaveDlg::coordsPrecision() const
{
	return coordsPrecisionSpinBox->value();
}

int AsciiSaveDlg::sfPrecision() const
{
	return sfPrecisionSpinBox->value();
}

bool AsciiSaveDlg::swapColorAndSF() const
{
	return orderComboBox->currentIndex() == 1;
}

void AsciiSaveDlg::initFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("AsciiSaveDialog");

	//read parameters
	bool saveColHeader		= settings.value("saveHeader", columnsHeaderCheckBox->isChecked()).toBool();
	bool savePtsHeader		= settings.value("savePtsHeader", pointCountHeaderCheckBox->isChecked()).toBool();
	int coordsPrecision		= settings.value("coordsPrecision", coordsPrecisionSpinBox->value()).toInt();
	int sfPrecision			= settings.value("sfPrecision", sfPrecisionSpinBox->value()).toInt();
	int separatorIndex		= settings.value("separator", separatorComboBox->currentIndex()).toInt();
	int orderIndex			= settings.value("saveOrder", orderComboBox->currentIndex()).toInt();

	//apply parameters
	columnsHeaderCheckBox->setChecked(saveColHeader);
	pointCountHeaderCheckBox->setChecked(savePtsHeader);
	coordsPrecisionSpinBox->setValue(coordsPrecision);
	sfPrecisionSpinBox->setValue(sfPrecision);
	separatorComboBox->setCurrentIndex(separatorIndex);
	orderComboBox->setCurrentIndex(orderIndex);

	settings.endGroup();
}

void AsciiSaveDlg::acceptAndSaveSettings()
{
	QSettings settings;
	settings.beginGroup("AsciiSaveDialog");

	//write parameters
	settings.setValue("saveHeader", columnsHeaderCheckBox->isChecked());
	settings.setValue("savePtsHeader", pointCountHeaderCheckBox->isChecked());
	settings.setValue("coordsPrecision", coordsPrecisionSpinBox->value());
	settings.setValue("sfPrecision", sfPrecisionSpinBox->value());
	settings.setValue("separator", separatorComboBox->currentIndex());
	settings.setValue("saveOrder", orderComboBox->currentIndex());

	settings.endGroup();
}

