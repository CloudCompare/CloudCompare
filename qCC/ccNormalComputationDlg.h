//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_NORMAL_COMPUTATION_DLG_HEADER
#define CC_NORMAL_COMPUTATION_DLG_HEADER

#include <ui_normalComputationDlg.h>

//qCC_db
#include <ccNormalVectors.h>

//CCLib
#include <CCConst.h> //for CC_LOCAL_MODEL_TYPES

class ccPointCloud;

//! Dialog for normal computation
class ccNormalComputationDlg : public QDialog, public Ui::NormalComputationDlg
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccNormalComputationDlg(QWidget* parent = 0);

	//! Returns local model chosen for normal computation
	CC_LOCAL_MODEL_TYPES getLocalModel() const;

	//! Sets default value for local neighbourhood radius
	void setRadius(PointCoordinateType radius);

	//! Sets the preferred orientation
	void setPreferredOrientation(ccNormalVectors::Orientation orientation);

	//! Sets the currently selected cloud (required for 'auto' feature)
	void setCloud(ccPointCloud* cloud);

	//! Returns local neighbourhood radius
	PointCoordinateType getRadius() const;

	//! Returns the preferred orientation (if any)
	ccNormalVectors::Orientation getPreferredOrientation() const;

protected slots:

	//! On local model change
	void localModelChanged(int index);

	//! Automatically estimate the local surface radius
	void autoEstimateRadius();

protected:

	//! Selected cloud
	ccPointCloud* m_cloud;

};

#endif // CC_NORMAL_COMPUTATION_DLG_HEADER
