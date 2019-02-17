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

//Local
#include "cc2.5DimEditor.h"

//Qt
#include <QDialog>
#include <QString>

class ccGenericPointCloud;
class ccPointCloud;
class ccPolyline;

namespace Ui
{
    class RasterizeToolDialog;
}

//! Rasterize tool (dialog)
class ccRasterizeTool : public QDialog, public cc2Point5DimEditor
{
	Q_OBJECT

public:
	//! Default constructor
	ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent = nullptr);

	//! Destructor
	~ccRasterizeTool() override;

public: //raster export

	//! Bands to be exported
	struct ExportBands
	{
		bool height = true;
		bool rgb = false;
		bool density = false;
		bool visibleSF = false;
		bool allSFs = false;
	};

	//! Exports a raster grid as a geotiff file
	static bool ExportGeoTiff(	const QString& outputFilename,
								const ExportBands& exportBands,
								ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy,
								const ccRasterGrid& grid,
								const ccBBox& gridBBox,
								unsigned char Z,
								double customHeightForEmptyCells = std::numeric_limits<double>::quiet_NaN(),
								ccGenericPointCloud* originCloud = nullptr,
								int visibleSfIndex = -1);

private:

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
	bool showGridBoxEditor() override;

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

private: //standard methods

	//Inherited from cc2Point5DimEditor
	double getGridStep() const override;
	unsigned char getProjectionDimension() const override;
	ccRasterGrid::ProjectionType getTypeOfProjection() const override;

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
	void gridIsUpToDate(bool state) override;

	//! Load persistent settings
	void loadSettings();

	//! Updates the grid
	bool updateGrid(bool interpolateSF = false);

	//! Tests if the dialog can be safely closed
	bool canClose();

	//! Adds a new contour line
	void addNewContour(ccPolyline* poly, double height, unsigned subIndex);

protected: //raster grid related stuff

	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertGridToCloud(	const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
										bool interpolateSF,
										bool interpolateColors,
										bool copyHillshadeSF,
										const QString& activeSFName,
										bool exportToOriginalCS) const;

private: //members

	//! Layer types
	enum LayerType {	LAYER_HEIGHT = 0,
						LAYER_RGB = 1,
						LAYER_SF = 2
	};

	Ui::RasterizeToolDialog* m_UI;
	
	//! Associated cloud
	ccGenericPointCloud* m_cloud;

	//! Contour lines
	std::vector<ccPolyline*> m_contourLines;
};

#endif //CC_RASTERIZE_TOOL_HEADER
