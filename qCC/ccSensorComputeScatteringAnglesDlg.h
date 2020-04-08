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

#include <QDialog>

namespace Ui {
	class sensorComputeScatteringAnglesDlg;
}

//! Dialog for scattering angles computation
class ccSensorComputeScatteringAnglesDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSensorComputeScatteringAnglesDlg(QWidget* parent = nullptr);
	
	~ccSensorComputeScatteringAnglesDlg();

	//! Returns whether angles should be converted to degrees
	bool anglesInDegrees() const;

private:
	Ui::sensorComputeScatteringAnglesDlg* m_ui;
};

#endif //CC_SF_SCATTERING_DLG_HEADER
