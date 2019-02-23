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

#ifndef BDR_3D4EM_DLG_HEADER
#define BDR_3D4EM_DLG_HEADER

#include "ui_bdr3D4EMDlg.h"
#include "mainwindow.h"

//! Dialog for qRansacSD plugin
class bdr3D4EMDlg : public QDialog, public Ui::BDR3D4EMDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdr3D4EMDlg(QWidget* parent = 0);

	int GroundHeightMode();
	double UserDefinedGroundHeight();
	double ground_height;

protected slots:

	void browsePointcloudFilename();
	void browseOutputFilename();
	void browseConfigureFilename();
	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();


};

#endif
