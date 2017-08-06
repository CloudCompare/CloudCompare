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

#ifndef CC_COMPUTE_OCTREE_DLG_HEADER
#define CC_COMPUTE_OCTREE_DLG_HEADER

#include <ui_computeOctreeDlg.h>

//qCC_db
#include <ccBBox.h>

class ccBoundingBoxEditorDlg;

//! Dialog for octree computation
class ccComputeOctreeDlg : public QDialog, public Ui::ComputeOctreeDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccComputeOctreeDlg(const ccBBox& baseBBox,
						double minCellSize,
						QWidget* parent = 0);

	//! Computation mode
	enum ComputationMode { DEFAULT, MIN_CELL_SIZE, CUSTOM_BBOX };

	//! Returns octree computation mode
	ComputationMode getMode() const;

	//! Returns cell size at max level
	double getMinCellSize() const;

	//! Returns custom bbox
	ccBBox getCustomBBox() const;

protected:

	//! Associated dialog
	ccBoundingBoxEditorDlg* m_bbEditorDlg;

};

#endif //CC_COMPUTE_OCTREE_DLG_HEADER
