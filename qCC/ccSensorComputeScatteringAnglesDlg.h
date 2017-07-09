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


#ifndef CC_SF_SCATTERING_DLG_HEADER
#define CC_SF_SCATTERING_DLG_HEADER

#include <ui_sensorComputeScatteringAnglesDlg.h>

//! Dialog for scattering angles computation
class ccSensorComputeScatteringAnglesDlg : public QDialog, public Ui::sensorComputeScatteringAnglesDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSensorComputeScatteringAnglesDlg(QWidget* parent = 0);

	//! Returns whether angles should be converted to degrees
	bool anglesInDegrees() const;

};

#endif //CC_SF_SCATTERING_DLG_HEADER
