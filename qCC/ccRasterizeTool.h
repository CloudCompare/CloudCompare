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

#ifndef CC_RASTERIZE_TOOL_HEADER
#define CC_RASTERIZE_TOOL_HEADER

#include <ui_rasterizeDlg.h>

//Local
#include "cc2.5DimEditor.h"

//Qt
#include <QString>

class ccGenericPointCloud;
class ccPointCloud;
class ccPolyline;

//! Rasterize tool (dialog)
class ccRasterizeTool : public QDialog, public cc2Point5DimEditor, public Ui::RasterizeToolDialog
{
	Q_OBJECT

public:
	//! Default constructor
	ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent = 0);

	//! Destructor
	~ccRasterizeTool();

protected slots:

	//! Exports the grid as a cloud
	ccPointCloud* generateCloud(bool autoExport = true) const;

	//! Exports the grid as a raster
	void generateRaster() const;

	//! Exports the grid as a mesh
	void generateMesh() const;

	//! Exports the (already generated) contour lines
	void exportContourLines();

	//! Generates a contour plot
	void generateContours();

	//! Generates hillshade
	void generateHillshade();

	//! Removes all displayed contour lines
	void removeContourLines();

	//! Accepts the dialog (if some conditions are met) and save settings
	void testAndAccept();
	//! Rejects the dialog (if some conditions are met)
	void testAndReject();

	//! Save persistent settings and 'accept' dialog
	void saveSettings();

	//! Called when the active layer changes
	void activeLayerChanged(int, bool autoRedraw = true);

	//! Called when the projection direction changes
	void projectionDirChanged(int);

	//! Called when the projection type changes
	void projectionTypeChanged(int);

	//! Called when the 'resampled' option changes
	void resampleOptionToggled(bool);

	//! Called when the SF projection type changes
	void sfProjectionTypeChanged(int);

	//Inherited from cc2Point5DimEditor
	virtual bool showGridBoxEditor();

	//! Called when the empty cell filling strategy changes
	void fillEmptyCellStrategyChanged(int);

	//! Called when the an option of the grid generation has changed
	void gridOptionChanged();

	//! Updates the gid info
	void updateGridInfo();

	//! Update the grid and the 2D display
	void updateGridAndDisplay();

	//! Exports the grid as an image
	virtual void generateImage() const;

	//! Exports the grid as an ASCII matrix
	virtual void generateASCIIMatrix() const;

protected: //standard methods

	//Inherited from cc2Point5DimEditor
	virtual double getGridStep() const override;
	virtual unsigned char getProjectionDimension() const override;
	virtual ccRasterGrid::ProjectionType getTypeOfProjection() const override;

	//! Returns user defined height for empty cells
	double getCustomHeightForEmptyCells() const;

	//! Returns strategy for empty cell filling (extended version)
	ccRasterGrid::EmptyCellFillOption getFillEmptyCellsStrategyExt(	double& emptyCellsHeight,
																	double& minHeight,
																	double& maxHeight) const;

	//! Returns whether a given field count should be exported as SF (only if a cloud is generated!)
	bool exportAsSF(ccRasterGrid::ExportableFields field) const;

	//! Returns whether the output cloud should use the original cloud or the grid as 'support'
	bool resampleOriginalCloud() const;

	//! Returns type of SF interpolation
	ccRasterGrid::ProjectionType getTypeOfSFInterpolation() const;

	//Inherited from cc2Point5DimEditor
	virtual void gridIsUpToDate(bool state);

	//! Load persistent settings
	void loadSettings();

	//! Updates the grid
	bool updateGrid(bool interpolateSF = false);

	//! Tests if the dialog can be safely closed
	bool canClose();

protected: //raster grid related stuff

	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertGridToCloud(	const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
										bool interpolateSF,
										bool interpolateColors,
										bool copyHillshadeSF,
										QString activeSFName,
										bool exportToOriginalCS) const;

protected: //members

	//! Layer types
	enum LayerType {	LAYER_HEIGHT = 0,
						LAYER_RGB = 1,
						LAYER_SF = 2
	};

	//! Associated cloud
	ccGenericPointCloud* m_cloud;

	//! Contour lines
	std::vector<ccPolyline*> m_contourLines;
};

#endif //CC_RASTERIZE_TOOL_HEADER
