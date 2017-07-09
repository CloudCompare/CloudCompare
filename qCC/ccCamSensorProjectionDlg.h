//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_CAMERA_SENSOR_PROJECTION_DIALOG_HEADER
#define CC_CAMERA_SENSOR_PROJECTION_DIALOG_HEADER

#include <ui_camSensorProjectDlg.h>

class ccCameraSensor;

//! Camera sensor parameters dialog
class ccCamSensorProjectionDlg : public QDialog, public Ui::CamSensorProjectDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccCamSensorProjectionDlg(QWidget* parent = 0);

	void initWithCamSensor(const ccCameraSensor* sensor);
	void updateCamSensor(ccCameraSensor* sensor);

};

#endif //CC_CAMERA_SENSOR_PROJECTION_DIALOG_HEADER
