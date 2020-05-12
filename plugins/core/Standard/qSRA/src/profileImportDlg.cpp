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

#include "profileImportDlg.h"

//Qt
#include <QFileDialog>

//System
#include <assert.h>

ProfileImportDlg::ProfileImportDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::ProfileImportDlg()
{
	setupUi(this);

	connect(browseToolButton, &QAbstractButton::clicked, this, &ProfileImportDlg::browseFile);
}

int ProfileImportDlg::getAxisDimension() const
{
	return axisDimComboBox->currentIndex();
}

void ProfileImportDlg::browseFile()
{
	QString filter("2D profile (*.txt)");

	//open file loading dialog
	QString filename = QFileDialog::getOpenFileName(nullptr,"Select profile file",getFilename(),filter
#if defined( Q_OS_WIN ) && defined( _DEBUG )
																,0,QFileDialog::DontUseNativeDialog
#endif
		
		);

	if (filename.isEmpty())
		return;
	
	setDefaultFilename(filename);
}

void ProfileImportDlg::setDefaultFilename(QString filename)
{
	inputFileLineEdit->setText(filename);
}

QString ProfileImportDlg::getFilename() const
{
	return inputFileLineEdit->text();
}

bool ProfileImportDlg::absoluteHeightValues() const
{
	return absoluteHeightValuesCheckBox->isChecked();
}
