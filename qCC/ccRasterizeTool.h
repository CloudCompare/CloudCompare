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

#ifndef CC_RASTERIZE_TOOL_HEADER
#define CC_RASTERIZE_TOOL_HEADER

#include <ui_rasterizeDlg.h>

//qCC_db
#include <ccBBox.h>

//Qt
#include <QString>
#include <QDialog>

//system
#include <vector>

class ccBoundingBoxEditorDlg;
class ccGenericPointCloud;
class ccPointCloud;
class ccGLWindow;

//! Rasterize tool (dialog)
class ccRasterizeTool : public QDialog, public Ui::RasterizeToolDialog
{
	Q_OBJECT

public:
	//! Default constructor
	ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent = 0);

	//! Destructor
	~ccRasterizeTool();

	//! Returns projection grid step
	double getGridStep() const;

	//! Returns whether a SF with per-cell count should be generated (only if a cloud is generated!)
	bool generateCountSF() const;

	//! Returns whether the output cloud should use the original cloud or the grid as 'support'
	bool resampleOriginalCloud() const;

	//! Returns projection dimension
	/** \return dimension as int (0: X, 1: Y, 2:Z)
	**/
	unsigned char getProjectionDimension() const;

	//! Types of projection
	enum ProjectionType {	PROJ_MINIMUM_VALUE			= 0,
							PROJ_AVERAGE_VALUE			= 1,
							PROJ_MAXIMUM_VALUE			= 2,
							INVALID_PROJECTION_TYPE		= 255,
	};

	//! Option for handling empty cells
	enum EmptyCellFillOption {	LEAVE_EMPTY				= 0,
								FILL_MINIMUM_HEIGHT		= 1,
								FILL_MAXIMUM_HEIGHT		= 2,
								FILL_CUSTOM_HEIGHT		= 3,
								FILL_AVERAGE_HEIGHT		= 4,
								INTERPOLATE				= 5,
	};

	//! Returns type of projection
	ProjectionType getTypeOfProjection() const;

	//! Returns type of SF interpolation
	ProjectionType getTypeOfSFInterpolation() const;

	//! Returns strategy for empty cell filling
	EmptyCellFillOption getFillEmptyCellsStrategy() const;
	
	//! Returns strategy for empty cell filling (extended version)
	EmptyCellFillOption getFillEmptyCellsStrategy(	double& emptyCellsHeight,
													double& minHeight,
													double& maxHeight) const;

	//! Returns user defined height for empty cells
	double getCustomHeightForEmptyCells() const;

	//! Returns custom bbox
	ccBBox getCustomBBox() const;

protected slots:

	//! Exports the grid as a cloud
	void generateCloud() const;

	//! Exports the grid as an image
	void generateImage() const;

	//! Exports the grid as a raster
	void generateRaster() const;

	//! Exports the grid as an ASCII matrix
	void generateASCIIMatrix() const;

	//! Save persistent settings and 'accept' dialog
	void saveSettings();

	//! Called when the projection direction changes
	void projectionDirChanged(int);

	//! Called when the projection type changes
	void projectionTypeChanged(int);

	//! Show grid box editor
	void showGridBoxEditor();

	//! Called when the empty cell filling strategy changes
	void fillEmptyCellStrategyChanged(int);

	//! Called when the an option of the grid generation has changed
	void gridOptionChanged();

	//! Updates the gid info
	void updateGridInfo();

	//! Update the grid and the 2D display
	void updateGridAndDisplay();

protected: //standard methods

	//! Declares whether the grid is up-to-date or not
	void gridIsUpToDate(bool state);

	//! Load persistent settings
	void loadSettings();

	//! Updates the 2D display zoom
	void update2DDisplayZoom(ccBBox& box);

	//! Updates the grid
	bool updateGrid(bool interpolateSF = false);

protected: //raster grid related stuff

	//! Raster grid cell
	struct RasterCell
	{
		//! Default constructor
		RasterCell()
			: height(0)
			, nbPoints(0)
			, pointIndex(0)
		{}

		//! Height value
		PointCoordinateType height;
		//! Number of points projected in this cell
		unsigned nbPoints;
		//! Nearest point index (if any)
		unsigned pointIndex;
	};

	//! Raster grid type
	struct RasterGrid
	{
		//! Default constructor
		RasterGrid()
			: width(0)
			, height(0)
			, gridStep(1.0)
			, minHeight(0)
			, maxHeight(0)
			, meanHeight(0)
			, nonEmptyCells(0)
			, valid(false)
		{}
		//! Destructor
		virtual ~RasterGrid() { clear(); }

		//! Initialiazes and reset the grid
		bool init(unsigned w, unsigned h);
		//! Release memory
		void clear();
		//! Reset all cells
		void reset();

		//! Sets valid
		inline void setValid(bool state) { valid = state; }
		//! Returns whether the grid is 'valid' or not
		inline bool isValid() const { return valid; }

		std::vector<RasterCell*> data;
		std::vector<double*> scalarFields;
		unsigned width;
		unsigned height;
		double gridStep;
		double minHeight;
		double maxHeight;
		double meanHeight;
		unsigned nonEmptyCells;
		bool valid;
	};

	//! Converts the grid to a scalar field
	ccPointCloud* convertGridToCloud(bool generateCountSF, bool interpolateSF) const;

protected: //members

	//! Associated dialog
	ccBoundingBoxEditorDlg* m_bbEditorDlg;

	//! Associated cloud
	ccGenericPointCloud* m_cloud;

	//! 2D display
	ccGLWindow* m_window;

	//! Grid
	RasterGrid m_grid;

	//! 'Raster' cloud
	ccPointCloud* m_rasterCloud;
};

#endif //CC_HEIGHT_GRID_GENERATION_DLG_HEADER
