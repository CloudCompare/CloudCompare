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

#ifndef CC_NORMAL_COMPUTATION_DLG_HEADER
#define CC_NORMAL_COMPUTATION_DLG_HEADER

#include <ui_normalComputationDlg.h>

//qCC_db
#include <ccNormalVectors.h>


class ccPointCloud;

//! Dialog for normal computation
class ccNormalComputationDlg : public QDialog, public Ui::NormalComputationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	/** \param withScanGrid whether the selection contains some structured point clouds
	 *  \param withSensor whether the selection contains some sensors associated to the point clouds
		\param parent parent widget
	**/
	explicit ccNormalComputationDlg(bool withScanGrid, bool withSensor, QWidget* parent = nullptr);

	//! Returns the local model chosen for normal computation
	CC_LOCAL_MODEL_TYPES getLocalModel() const;

	//! Sets the local model chosen for normal computation
	void setLocalModel(CC_LOCAL_MODEL_TYPES  model);

	//! Sets default value for local neighbourhood radius
	void setRadius(PointCoordinateType radius);

	//! Sets the preferred orientation
	void setPreferredOrientation(ccNormalVectors::Orientation orientation);

	//! Sets the currently selected cloud (required for 'auto' feature)
	void setCloud(ccPointCloud* cloud);

	//! Returns whether scan grids should be used for computation
	bool useScanGridsForComputation() const;

	//! Returns the min angle for grid triangles
	double getMinGridAngle_deg() const;

	//! Sets the min angle for grid triangles
	void setMinGridAngle_deg(double value);

	//! Returns local neighbourhood radius
	PointCoordinateType getRadius() const;

	//! Returns whether normals should be oriented or not
	bool orientNormals() const;

	//! Returns whether scan grids should be used for normals orientation
	bool useScanGridsForOrientation() const;

	//! Returns whether scan grids should be used for normals computation
	bool useSensorsForOrientation() const;

	//! Returns whether a preferred orientation should be used
	bool usePreferredOrientation() const;

	//! Returns the preferred orientation (if any)
	ccNormalVectors::Orientation getPreferredOrientation() const;

	//! Returns whether a Minimum Spanning Tree (MST) should be used for normals orientation
	bool useMSTOrientation() const;

	//! Returns the number of neighbors for Minimum Spanning Tree (MST)
	int getMSTNeighborCount() const;

	//! Sets the number of neighbors for Minimum Spanning Tree (MST)
	void setMSTNeighborCount(int n);

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
