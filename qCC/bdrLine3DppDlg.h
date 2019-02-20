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

#ifndef BDR_LINE3DPP_DLG_HEADER
#define BDR_LINE3DPP_DLG_HEADER

#include "ui_bdrLine3DppDlg.h"
#include "mainwindow.h"

//! Dialog for qRansacSD plugin
class bdrLine3DppDlg : public QDialog, public Ui::BDRLine3DppDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrLine3DppDlg(QWidget* parent = 0);

protected slots:

	//! Generate Lines on acceptation
	void GenerateLines();

protected:
	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();

	MainWindow* m_win;
};

#endif
