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

#ifndef CC_HEIGHT_GRID_GENERATION_DLG_HEADER
#define CC_HEIGHT_GRID_GENERATION_DLG_HEADER

#include <ui_heightGridGenerationDlg.h>

//Local
#include "ccHeightGridGeneration.h"

//qCC_db
#include <ccBBox.h>

//Qt
#include <QString>

class ccBoundingBoxEditorDlg;

//! Height grid generation algorithm dialog
class ccHeightGridGenerationDlg : public QDialog, public Ui::HeightGridGenerationDialog
{
	Q_OBJECT

public:
	//! Default constructor
	ccHeightGridGenerationDlg(const ccBBox& gridBBox, QWidget* parent = 0);

	//! Returns projection grid step
	double getGridStep() const;

	//! Returns whether grid should be converted to a cloud
	bool generateCloud() const;

	//! Returns whether a SF with per-cell count should be generated (only if a cloud is generated!)
	bool generateCountSF() const;

	//! Returns whether the output cloud should use the original cloud or the grid as 'support'
	bool resampleOriginalCloud() const;

	//! Returns whether grid should be converted to an image
	bool generateImage() const;

	//! Returns whether grid should be converted to a raster
	bool generateRaster() const;

	//! Returns whether grid should be converted to an ASCII file
	bool generateASCII() const;

	//! Returns projection dimension
	/** \return dimension as int (0: X, 1: Y, 2:Z)
	**/
	unsigned char getProjectionDimension() const;

	//! Returns type of projection
	ccHeightGridGeneration::ProjectionType getTypeOfProjection() const;

	//! Returns type of SF interpolation
	ccHeightGridGeneration::ProjectionType getTypeOfSFInterpolation() const;

	//! Returns strategy for empty cell filling
	ccHeightGridGeneration::EmptyCellFillOption getFillEmptyCellsStrategy() const;

	//! Returns user defined height for empty cells
	double getCustomHeightForEmptyCells() const;

	//! Returns custom bbox
	ccBBox getCustomBBox() const;

protected slots:

	//! Save persistent settings and 'accept' dialog
	void saveSettings();

	//! Slot to handle projection type change
	void projectionChanged(int);

	//! Updates the "fill empty cells" frame state
	void toggleFillEmptyCells(bool);

	//! Show grid box editor
	void showGridBoxEditor();

	//! Called when the projection type changes
	void projectionTypeChanged(int);

protected:

	//! Load persistent settings
	void loadSettings();

	//! Associated dialog
	ccBoundingBoxEditorDlg* m_bbEditorDlg;
};

#endif //CC_HEIGHT_GRID_GENERATION_DLG_HEADER
