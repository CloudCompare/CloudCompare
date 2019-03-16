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

#ifndef CC_RANSAC_SD_DLG_HEADER
#define CC_RANSAC_SD_DLG_HEADER

#include "ui_ransacSDDlg.h"

//! Dialog for qRansacSD plugin
class ccRansacSDDlg : public QDialog, public Ui::RansacSDDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccRansacSDDlg(QWidget* parent = 0);

protected slots:

	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();

};

#endif
