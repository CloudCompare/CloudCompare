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

#ifndef BDR_PLANESEG_DLG_HEADER
#define BDR_PLANESEG_DLG_HEADER

#include "ui_bdrPlaneSegDlg.h"
#include "mainwindow.h"

//! Dialog for qRansacSD plugin
class bdrPlaneSegDlg : public QDialog, public Ui::BDRPlaneSegDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPlaneSegDlg(QWidget* parent = 0);
	void setPointClouds(ccHObject::Container point_clouds);

protected slots:
	void DeducePara();
	void exitSafe();
	void Execute();

protected:
	ccHObject::Container m_point_clouds;
};

#endif
