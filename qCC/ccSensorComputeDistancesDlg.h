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

#include <QDialog>

namespace Ui {
	class sensorComputeDistancesDlg;
}

//! Dialog for sensor range computation
class ccSensorComputeDistancesDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSensorComputeDistancesDlg(QWidget* parent = nullptr);
	
	~ccSensorComputeDistancesDlg();

	//! Returns whether computed distances should be squared or not
	bool computeSquaredDistances() const;

private:
	Ui::sensorComputeDistancesDlg* m_ui;
};

#endif //CC_SF_DISTANCES_DLG_HEADER
