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

#ifndef BDR_POLYFIT_DLG_HEADER
#define BDR_POLYFIT_DLG_HEADER

#include "ui_bdrPolyFitDlg.h"
#include "mainwindow.h"

//! Dialog for qRansacSD plugin
class bdrPolyFitDlg : public QDialog, public Ui::BDRPolyFitDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPolyFitDlg(QWidget* parent = 0);

protected slots:
	 
};

#endif
