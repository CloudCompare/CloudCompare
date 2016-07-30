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

#ifndef CC_2_5D_EDITOR_HEADER
#define CC_2_5D_EDITOR_HEADER

#include <ui_rasterizeDlg.h>

//qCC_db
#include <ccBBox.h>

//system
#include <limits>

class ccBoundingBoxEditorDlg;
class ccGenericPointCloud;
class ccPointCloud;
class ccProgressDialog;
class ccGLWindow;
class QWidget;
class QFrame;
class QComboBox;

//! 2.5D data editor (generic interface)
class cc2Point5DimEditor
{
public:
	//! Default constructor
	cc2Point5DimEditor();

	//! Destructor
	virtual ~cc2Point5DimEditor();

	//! Exportable fields
	enum ExportableFields { PER_CELL_HEIGHT,
							PER_CELL_COUNT,
							PER_CELL_MIN_HEIGHT,
							PER_CELL_MAX_HEIGHT,
							PER_CELL_AVG_HEIGHT,
							PER_CELL_HEIGHT_STD_DEV,
							PER_CELL_HEIGHT_RANGE,
							PER_CELL_INVALID,
	};

	//! Returns the default name of a given field
	static QString GetDefaultFieldName(ExportableFields field);

protected: //standard methods

	//! Returns projection grid step
	virtual double getGridStep() const = 0;

	//! Returns projection dimension
	/** \return dimension as int (0: X, 1: Y, 2:Z)
	**/
	virtual unsigned char getProjectionDimension() const = 0;

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
	virtual ProjectionType getTypeOfProjection() const = 0;

	//! Returns custom bbox
	virtual ccBBox getCustomBBox() const;

	//! Declares whether the grid is up-to-date or not
	virtual void gridIsUpToDate(bool state) = 0;

	//! Updates the 2D display zoom
	virtual void update2DDisplayZoom(ccBBox& box);

protected: //raster grid related stuff

	//! Show grid box editor and update 
	/** \return whether the box was modified or not
	**/
	virtual bool showGridBoxEditor();

	//! Returns the grid size as a string
	virtual QString getGridSizeAsString() const;

	//! Returns the grid size
	virtual bool getGridSize(unsigned& width, unsigned& height) const;

	//! Creates the bounding-box editor
	void createBoundingBoxEditor(const ccBBox& gridBBox, QWidget* parent);

	//! Creates the 2D view
	void create2DView(QFrame* parentFrame);

	//! Returns the empty cell strategy (for a given combo-box)
	EmptyCellFillOption getFillEmptyCellsStrategy(QComboBox* comboBox) const;

	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertGridToCloud(	const std::vector<ExportableFields>& exportedFields,
										bool interpolateSF,
										bool interpolateColors,
										bool resampleInputCloudXY,
										bool resampleInputCloudZ, //only considered if resampleInputCloudXY is true!
										ccGenericPointCloud* inputCloud,
										bool fillEmptyCells,
										double emptyCellsHeight) const;

	//! Raster grid cell
	struct RasterCell
	{
		//! Default constructor
		RasterCell()
			: h(std::numeric_limits<double>::quiet_NaN())
			, avgHeight(0)
			, stdDevHeight(0)
			, minHeight(0)
			, maxHeight(0)
			, nbPoints(0)
			, pointIndex(0)
			, color(0, 0, 0)
		{}

		//! Height value
		double h;
		//! Average height value
		double avgHeight;
		//! Height std.dev.
		double stdDevHeight;
		//! Min height value
		PointCoordinateType minHeight;
		//! Max height value
		PointCoordinateType maxHeight;
		//! Number of points projected in this cell
		unsigned nbPoints;
		//! Nearest point index (if any)
		unsigned pointIndex;
		//! Color
		CCVector3d color;
	};

	//! Raster grid type
	struct RasterGrid
	{
		//! Default constructor
		RasterGrid()
			: width(0)
			, height(0)
			, gridStep(1.0)
			, minCorner(0,0,0)
			, minHeight(0)
			, maxHeight(0)
			, meanHeight(0)
			, nonEmptyCellCount(0)
			, validCellCount(0)
			, hasColors(false)
			, valid(false)
		{}
		//! Destructor
		virtual ~RasterGrid() { clear(); }

		//! Initializes / resets the grid
		bool init(	unsigned w,
					unsigned h,
					double gridStep,
					const CCVector3d& minCorner);

		//! Clears the grid
		void clear();

		//! Fills the grid with a point cloud
		/** Since version 2.8, we now use the "PixelIsArea" convention by default (as GDAL)
			This means that the height is computed at the center of the grid cell.
		**/
		bool fillWith(	ccGenericPointCloud* cloud,
						unsigned char projectionDimension,
						cc2Point5DimEditor::ProjectionType projectionType,
						bool interpolateEmptyCells,
						cc2Point5DimEditor::ProjectionType sfInterpolation = INVALID_PROJECTION_TYPE,
						ccProgressDialog* progressDialog = 0);

		//! Fills the empty cell (for all strategies but 'INTERPOLATE')
		void fillEmptyGridCells(cc2Point5DimEditor::EmptyCellFillOption fillEmptyCellsStrategy,
								double customCellHeight = 0);

		//! Sets valid
		inline void setValid(bool state) { valid = state; }
		//! Returns whether the grid is 'valid' or not
		inline bool isValid() const { return valid; }

		//! Row
		typedef std::vector<RasterCell> Row;
		
		//! All cells
		std::vector<Row> rows;

		//! Scalar field
		typedef std::vector<double> SF;

		//! Associated scalar fields
		std::vector<SF> scalarFields;
		
		//! Number of columns
		unsigned width;
		//! Number of rows
		unsigned height;
		//! Grid step ('pixel' size)
		double gridStep;
		//! Min corner (3D)
		CCVector3d minCorner;
		
		//! Min height (computed on the NON-EMPTY or INTERPOLATED cells)
		double minHeight;
		//! Max height (computed on the NON-EMPTY or INTERPOLATED cells)
		double maxHeight;
		//! Average height (computed on the NON-EMPTY or INTERPOLATED cells)
		double meanHeight;
		//! Number of NON-EMPTY cells
		unsigned nonEmptyCellCount;
		//! Number of VALID cells
		unsigned validCellCount;

		//! Whether the (average) colors are available or not
		bool hasColors;
		
		//! Whether the grid is valid/up-to-date
		bool valid;
	};

protected: //members

	//! Associated dialog
	ccBoundingBoxEditorDlg* m_bbEditorDlg;

	//! 2D display
	ccGLWindow* m_glWindow;

	//! Grid
	RasterGrid m_grid;

	//! 'Raster' cloud
	ccPointCloud* m_rasterCloud;
};

#endif //CC_2_5D_EDITOR_HEADER
