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

#ifndef CC_SF_DISTANCES_DLG_HEADER
#define CC_SF_DISTANCES_DLG_HEADER

#include <ui_sensorComputeDistancesDlg.h>

//! Dialog for sensor range computation
class ccSensorComputeDistancesDlg : public QDialog, public Ui::sensorComputeDistancesDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSensorComputeDistancesDlg(QWidget* parent = 0);

	//! Returns whether computed distances should be squared or not
	bool computeSquaredDistances() const;

};

#endif //CC_SF_DISTANCES_DLG_HEADER
