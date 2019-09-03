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

#include "ui_bdrSettingLoD2Dlg.h"
#include "mainwindow.h"

//! Dialog for qRansacSD plugin
class bdrSettingLoD2Dlg : public QDialog, public Ui::bdrSettingLoD2Dlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrSettingLoD2Dlg(QWidget* parent = 0);

	int GroundHeightMode();
	double UserDefinedGroundHeight();
	double ground_height;

protected slots:

	void browseConfigureFilename();
	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();


};

#endif
