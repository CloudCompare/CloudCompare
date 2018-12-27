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

#ifndef CC_RASTER_GRID_HEADER
#define CC_RASTER_GRID_HEADER

//local
#include "qCC_db.h"
#include "ccBBox.h"

//system
#include <limits>

class ccGenericPointCloud;
class ccPointCloud;
class ccProgressDialog;

//! Raster grid cell
struct QCC_DB_LIB_API ccRasterCell
{
	//! Default constructor
	ccRasterCell()
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
struct QCC_DB_LIB_API ccRasterGrid
{
	//! Default constructor
	ccRasterGrid();
	
	//! Destructor
	virtual ~ccRasterGrid();

	//! Computes the raster size for a given bounding-box
	static bool ComputeGridSize(unsigned char Z,
								const ccBBox& box,
								double gridStep,
								unsigned& width,
								unsigned& height);


	//! Initializes / resets the grid
	bool init(	unsigned w,
				unsigned h,
				double gridStep,
				const CCVector3d& minCorner);

	//! Clears the grid
	void clear();

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

	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertToCloud(	const std::vector<ExportableFields>& exportedFields,
									bool interpolateSF,
									bool interpolateColors,
									bool resampleInputCloudXY,
									bool resampleInputCloudZ, //only considered if resampleInputCloudXY is true!
									ccGenericPointCloud* inputCloud,
									unsigned char Z,
									const ccBBox& box,
									bool fillEmptyCells,
									double emptyCellsHeight,
									bool exportToOriginalCS) const;

	//! Types of projection
	enum ProjectionType {	PROJ_MINIMUM_VALUE			= 0,
							PROJ_AVERAGE_VALUE			= 1,
							PROJ_MAXIMUM_VALUE			= 2,
							INVALID_PROJECTION_TYPE		= 255,
	};

	//! Fills the grid with a point cloud
	/** Since version 2.8, we now use the "PixelIsArea" convention by default (as GDAL)
	This means that the height is computed at the center of the grid cell.
	**/
	bool fillWith(	ccGenericPointCloud* cloud,
					unsigned char projectionDimension,
					ProjectionType projectionType,
					bool interpolateEmptyCells,
					ProjectionType sfInterpolation = INVALID_PROJECTION_TYPE,
					ccProgressDialog* progressDialog = nullptr);

	//! Option for handling empty cells
	enum EmptyCellFillOption {	LEAVE_EMPTY				= 0,
								FILL_MINIMUM_HEIGHT		= 1,
								FILL_MAXIMUM_HEIGHT		= 2,
								FILL_CUSTOM_HEIGHT		= 3,
								FILL_AVERAGE_HEIGHT		= 4,
								INTERPOLATE				= 5,
	};

	//! Fills the empty cell (for all strategies but 'INTERPOLATE')
	void fillEmptyCells(EmptyCellFillOption fillEmptyCellsStrategy,
						double customCellHeight = 0);

	//! Sets valid
	inline void setValid(bool state) { valid = state; }
	//! Returns whether the grid is 'valid' or not
	inline bool isValid() const { return valid; }

	//! Computes the position of the cell that includes a given point
	std::pair<int, int> computeCellPos(const CCVector3& P, unsigned char X, unsigned char Y) const
	{
		CCVector3d relativePos = CCVector3d::fromArray(P.u) - minCorner;

		//DGM: we use the 'PixelIsArea' convention
		int i = static_cast<int>((relativePos.u[X] / gridStep + 0.5));
		int j = static_cast<int>((relativePos.u[Y] / gridStep + 0.5));

		return {i, j};
	}

	//! Computes the position of the center of a given cell
	CCVector2d computeCellCenter(int i, int j, unsigned char X, unsigned char Y) const
	{
		return {minCorner.u[X] + (i + 0.5) * gridStep, minCorner.u[Y] + (j + 0.5) * gridStep};
	}

	//! Row
	using Row = std::vector<ccRasterCell>;

	//! All cells
	std::vector<Row> rows;

	//! Scalar field
	using SF = std::vector<double>;

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

#endif //CC_RASTER_GRID_HEADER
