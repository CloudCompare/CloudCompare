//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "bdrSettingLoD2Dlg.h"

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrSettingLoD2Dlg::bdrSettingLoD2Dlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::bdrSettingLoD2Dlg()
{
	setupUi(this);

	connect(ConfigureFilePathToolButton, &QAbstractButton::clicked, this, &bdrSettingLoD2Dlg::browseConfigureFilename);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}

int bdrSettingLoD2Dlg::GroundHeightMode()
{
	if (GroundHeightLowestRadioButton->isChecked()) {
		return 0;
	}
	else if (GroundHeightContourRadioButton->isChecked()) {
		return 1;
	}
	else if (GroundHeightUserRadioButton->isChecked()) {
		return 2;
	}
	return 0;
}

double bdrSettingLoD2Dlg::UserDefinedGroundHeight()
{
	return GroundHeightUserDoubleSpinBox->value();
}

void bdrSettingLoD2Dlg::browseConfigureFilename()
{
	QString Filename =
		QFileDialog::getOpenFileName(this,
			"Open Configuration file",
			ConfigureFilePathLineEdit->text(),
			"Configuration (*.ini)");

	if (!Filename.isEmpty())
		ConfigureFilePathLineEdit->setText(Filename);
}

void bdrSettingLoD2Dlg::saveSettings()
{
	ConfigureFilePathLineEdit;
}