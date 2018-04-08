//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qCSVMatrixIO                     #
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

#ifndef CSV_MATRIX_OPEN_DIALOG_HEADER
#define CSV_MATRIX_OPEN_DIALOG_HEADER

//Qt
#include <QDialog>

//GUI
#include "ui_openCSVMatrixDlg.h"

//! CSV Matrix Open dialog
class CSVMatrixOpenDialog : public QDialog, public Ui::CSVMatrixOpenDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit CSVMatrixOpenDialog(QWidget* parent = 0);

protected slots:

	//! Bowse texture file
	void browseTextureFile();
};

#endif
