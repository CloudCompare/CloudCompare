#pragma once

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

//local
#include "qCC_db.h"
#include "ccBBox.h"

//CCCoreLib
#include <Kriging.h>

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
		, minHeight(0)
		, maxHeight(0)
		, nbPoints(0)
		, nearestPointIndex(0)
		, color(0, 0, 0)
		, pointRefHead(nullptr)
		, pointRefTail(nullptr)
	{}

	//! Returns the list of all point indexes projected into this cell
	void getPointIndexes(std::vector<unsigned>& indexes, const std::vector<void*>& pointRefList) const;

	//! Height value
	double h;
	//! Min height value
	PointCoordinateType minHeight;
	//! Max height value
	PointCoordinateType maxHeight;
	//! Number of points projected in this cell
	unsigned nbPoints;
	//! Nearest point index (if any)
	unsigned nearestPointIndex;
	//! Color
	CCVector3d color;
	//! Pointer to first point reference for this cell (used to compute the median)
	void** pointRefHead;
	//! Pointer to last point reference for this cell (used to compute the median)
	void** pointRefTail;

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
	/** We use the "Pixel-is-area" convention but 'min corner'
		corresponds to the lower left cell CENTER.
		-------------------
		|     |     |     |
		|  U  |  V  |  W  |
		|     |     |     |
		-------------------
		|     |     |     |
		|  X  |  Y  |  Z  |
		|     |     |     |
		-------------------

		Here, w=3 and h=2, and minCorner=X

	**/
	bool init(	unsigned w,
				unsigned h,
				double gridStep,
				const CCVector3d& minCorner);

	//! Clears the grid
	void clear();

	//! Resets the grid
	void reset();

	//! Exportable fields
	enum ExportableFields { PER_CELL_VALUE,
							PER_CELL_COUNT,
							PER_CELL_MIN_VALUE,
							PER_CELL_MAX_VALUE,
							PER_CELL_AVG_VALUE,
							PER_CELL_VALUE_STD_DEV,
							PER_CELL_VALUE_RANGE,
							PER_CELL_MEDIAN_VALUE,
							PER_CELL_PERCENTILE_VALUE,
							PER_CELL_UNIQUE_COUNT_VALUE,
							PER_CELL_INVALID,
	};

	//! Returns the default name of a given field
	static QString GetDefaultFieldName(ExportableFields field);

	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertToCloud(	bool exportHeightStats,
									bool exportSFStats,
									const std::vector<ExportableFields>& exportedStatistics,
									bool projectSFs,
									bool projectColors,
									bool resampleInputCloudXY,
									bool resampleInputCloudZ, //only considered if resampleInputCloudXY is true!
									ccGenericPointCloud* inputCloud,
									unsigned char Z,
									const ccBBox& box,
									double percentileValue,
									bool exportToOriginalCS,
									bool appendGridSizeToSFNames,
									ccProgressDialog* progressDialog = nullptr ) const;

	//! Types of projection
	enum ProjectionType {	PROJ_MINIMUM_VALUE			= 0,
							PROJ_AVERAGE_VALUE			= 1,
							PROJ_MAXIMUM_VALUE			= 2,
							PROJ_MEDIAN_VALUE			= 3,
							PROJ_INVERSE_VAR_VALUE		= 4,
							INVALID_PROJECTION_TYPE		= 255,
	};

	//! Types of interpolation
	enum class InterpolationType {	NONE = 0,
									DELAUNAY = 1,
									KRIGING = 2
	};

	//! Delaunay interpolation parameter(s)
	struct QCC_DB_LIB_API DelaunayInterpolationParams
	{
		double maxEdgeLength = 1.0;
	};

	//! Kriging parameters
	struct QCC_DB_LIB_API KrigingParams
	{
		Kriging::KrigeParams params;
		bool autoGuess = true;
		int kNN = 8;
	};

	//! Fills the grid with a point cloud
	/** Since version 2.8, we are using the "PixelIsPoint" convention
		(contrarily to what was written in the code comments so far!).
		This means that the height is computed at the center of the grid cell.
	**/
	bool fillWith(	ccGenericPointCloud* cloud,
					unsigned char projectionDimension,
					ProjectionType projectionType,
					InterpolationType emptyCellsInterpolation = InterpolationType::NONE,
					void* interpolationParams = nullptr, // either nullptr, DelaunayInterpolationParams* or KrigingParams*
					ProjectionType sfProjectionType = INVALID_PROJECTION_TYPE,
					ccProgressDialog* progressDialog = nullptr,
					int zStdDevSfIndex = -1);

	//! Option for handling empty cells
	enum EmptyCellFillOption {	LEAVE_EMPTY				= 0,
								FILL_MINIMUM_HEIGHT		= 1,
								FILL_MAXIMUM_HEIGHT		= 2,
								FILL_CUSTOM_HEIGHT		= 3,
								FILL_AVERAGE_HEIGHT		= 4,
								INTERPOLATE_DELAUNAY	= 5,
								KRIGING					= 6,
	};

	//! Converts the empty cells fill option to the corresponding interpolation type
	static InterpolationType InterpolationTypeFromEmptyCellFillOption(EmptyCellFillOption option);

	//! Fills the empty cells
	void fillEmptyCells(EmptyCellFillOption fillEmptyCellsStrategy,
						double customCellHeight = 0.0);

	//! Updates the number of non-empty cells
	unsigned updateNonEmptyCellCount();

	//! Updates the statistics about the cells
	void updateCellStats();

	//! Interpolates the empty cells
	/** \warning The number of non empty cells must be up-to-date (see updateNonEmptyCellCount)
		\param maxSquareEdgeLength Max (square) edge length to filter large triangles during the interpolation process
	**/
	bool interpolateEmptyCells(double maxSquareEdgeLength);

	//! Interpolates the empty cells with the Kriging algorithm
	bool fillGridCellsWithKriging(	unsigned char Z,
									int knn,
									Kriging::KrigeParams& krigeParams,
									bool useInputParams,
									ccProgressDialog* progressDialog = nullptr);

	//! Sets valid
	inline void setValid(bool state) { valid = state; }
	//! Returns whether the grid is 'valid' or not
	inline bool isValid() const { return valid; }

	//! Computes the position of the cell that includes a given point
	inline CCVector2i computeCellPos(const CCVector3& P, unsigned char dimX, unsigned char dimY) const
	{
		//minCorner corresponds to the lower left cell CENTER
		return CCVector2i(	static_cast<int>((P.u[dimX] - minCorner.u[dimX]) / gridStep + 0.5),
							static_cast<int>((P.u[dimY] - minCorner.u[dimY]) / gridStep + 0.5) );
	}

	//! Computes the position of the center of a given cell
	inline CCVector2d computeCellCenter(int i, int j, unsigned char dimX, unsigned char dimY) const
	{
		//minCorner corresponds to the lower left cell CENTER
		return CCVector2d(minCorner.u[dimX] + i * gridStep, minCorner.u[dimY] + j * gridStep);
	}

	//! Row
	using Row = std::vector<ccRasterCell>;

	//! All cells
	std::vector<Row> rows;

	//! Scalar field
	using SF = std::vector<double>;

	//! Associated scalar fields
	std::vector<SF> scalarFields;
	
    //! Array of pointers, each coresponding to a point in the cloud
	/** 'cloud->getPoint(n)' coresponds to 'pointRefList[n]'
		The pointers are used to chain points belonging to the same cell together.
	**/
	std::vector<void*> pointRefList;

	//! Number of columns
	unsigned width;
	//! Number of rows
	unsigned height;
	//! Grid step ('pixel' size)
	double gridStep;
	//! Min corner (3D)
	CCVector3d minCorner;

	//! Min height (computed on the NON-EMPTY or FILLED/INTERPOLATED cells)
	double minHeight;
	//! Max height (computed on the NON-EMPTY or FILLED/INTERPOLATED cells)
	double maxHeight;
	//! Average height (computed on the NON-EMPTY or FILLED/INTERPOLATED cells)
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
