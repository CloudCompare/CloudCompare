//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qKinect                       #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1595                                                              $
//$LastChangedDate:: 2010-07-02 18:04:17 +0200 (ven., 02 juil. 2010)       $
//**************************************************************************
//

#ifndef CC_KINECT_DLG_HEADER
#define CC_KINECT_DLG_HEADER

#include "ui_kinectDlg.h"

//! Dialog for qKinect plugin
class ccKinectDlg : public QDialog, public Ui::KinectDialog
{
public:

	//! Default constructor
    ccKinectDlg(QWidget* parent=0);

	//! Returns output clound name
	QString getCloudName() const;

	//! Returns output clound name
	bool grabRGBInfo();

	//! Adds 'resolution mode' string
	void addMode(const QString& mode);

	//! Returns frame averaging
	unsigned char getFrameAveragingCount() const;
};

#endif
