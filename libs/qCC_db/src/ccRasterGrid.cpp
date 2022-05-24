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

#include "ccRasterGrid.h"

//CCCoreLib
#include <Delaunay2dMesh.h>

//qCC_db
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "ccScalarField.h"

//Qt
#include <QCoreApplication>
#include <QMap>

//System
#include <cassert>

//default field names
struct DefaultFieldNames : public QMap<ccRasterGrid::ExportableFields, QString>
{
	DefaultFieldNames()
	{
		insert(ccRasterGrid::PER_CELL_HEIGHT,         "Height grid values");
		insert(ccRasterGrid::PER_CELL_COUNT,          "Per-cell population");
		insert(ccRasterGrid::PER_CELL_MIN_HEIGHT,     "Min height");
		insert(ccRasterGrid::PER_CELL_MAX_HEIGHT,     "Max height");
		insert(ccRasterGrid::PER_CELL_AVG_HEIGHT,     "Average height");
		insert(ccRasterGrid::PER_CELL_HEIGHT_STD_DEV, "Std. dev. height");
		insert(ccRasterGrid::PER_CELL_HEIGHT_RANGE,   "Height range");
		insert(ccRasterGrid::PER_CELL_MEDIAN_HEIGHT,  "Median height");
	}
};
static DefaultFieldNames s_defaultFieldNames;

QString ccRasterGrid::GetDefaultFieldName(ExportableFields field)
{
	assert(s_defaultFieldNames.contains(field));
	return s_defaultFieldNames[field];
}

ccRasterGrid::ccRasterGrid()
	: width(0)
	, height(0)
	, gridStep(1.0)
	, minCorner(0, 0, 0)
	, minHeight(0)
	, maxHeight(0)
	, meanHeight(0)
	, nonEmptyCellCount(0)
	, validCellCount(0)
	, hasColors(false)
	, valid(false)
{}

ccRasterGrid::~ccRasterGrid()
{
	clear();
}

bool ccRasterGrid::ComputeGridSize(	unsigned char Z,
									const ccBBox& box,
									double gridStep,
									unsigned& gridWidth,
									unsigned& gridHeight)
{
	gridWidth = gridHeight = 0;

	if (Z > 2 || !box.isValid() || gridStep <= 0)
	{
		//invalid input
		assert(false);
		ccLog::Warning("[ccRasterGrid::ComputeGridSize] Invalid input");
		return false;
	}

	//vertical dimension
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	CCVector3d boxDiag = box.maxCorner().toDouble() - box.minCorner().toDouble();
	if (boxDiag.u[X] <= 0 || boxDiag.u[Y] <= 0)
	{
		ccLog::Warning("[ccRasterGrid::ComputeGridSize] Invalid cloud bounding box!");
		return false;
	}

	//DGM: we use the 'PixelIsArea' convention but minCorner is the lower left cell CENTER
	gridWidth = 1 + static_cast<unsigned>(boxDiag.u[X] / gridStep + 0.5);
	gridHeight = 1 + static_cast<unsigned>(boxDiag.u[Y] / gridStep + 0.5);

	return true;
}

void ccRasterGrid::clear()
{
	//clear
	width = height = 0;
	rows.resize(0);
	scalarFields.resize(0);

	minHeight = maxHeight = meanHeight = 0;
	nonEmptyCellCount = validCellCount = 0;
	hasColors = false;

	setValid(false);
}

void ccRasterGrid::reset()
{
	for (Row& row : rows)
	{
		std::fill(row.begin(), row.end(), ccRasterCell());
	}

	minHeight = maxHeight = meanHeight = 0;
	nonEmptyCellCount = validCellCount = 0;
	hasColors = false;

	setValid(false);
}

bool ccRasterGrid::init(unsigned w,
						unsigned h,
						double s,
						const CCVector3d& c)
{
	//we always restart from scratch (clearer / safer)
	clear();

	try
	{
		rows.resize(h);
		for (Row& row : rows)
		{
			row.resize(w);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		rows.resize(0);
		return false;
	}

	width = w;
	height = h;
	gridStep = s;
	minCorner = c;

	return true;
}

//! Index and value
struct IndexAndValue
{
	unsigned index = 0;
	double val = 0.0;
};


bool ccRasterGrid::fillWith(	ccGenericPointCloud* cloud,
								unsigned char Z,
								ProjectionType projectionType,
								bool doInterpolateEmptyCells,
								double maxEdgeLength,
								ProjectionType sfInterpolation/*=INVALID_PROJECTION_TYPE*/,
								ccProgressDialog* progressDialog/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	unsigned gridTotalSize = width * height;
	if (gridTotalSize == 0)
	{
		assert(false);
		return false;
	}

	//input cloud as a ccPointCloud
	ccPointCloud* pc = nullptr;
	if (cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		pc = static_cast<ccPointCloud*>(cloud);
	}

	//do we need to interpolate scalar fields?
	bool interpolateSF = (sfInterpolation != INVALID_PROJECTION_TYPE);
	if (interpolateSF)
	{
		if (pc && pc->hasScalarFields())
		{
			unsigned sfCount = pc->getNumberOfScalarFields();
			try
			{
				scalarFields.resize(sfCount);
				for (unsigned i = 0; i < sfCount; ++i)
				{
					scalarFields[i].resize(gridTotalSize, std::numeric_limits<SF::value_type>::quiet_NaN());
				}
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				scalarFields.resize(0);
				ccLog::Warning("[Rasterize] Failed to allocate memory for scalar fields!");
			}
		}
		else
		{
			interpolateSF = false;
		}
	}

	//filling the grid
	unsigned pointCount = cloud->size();

	if (progressDialog)
	{
		progressDialog->setMethodTitle(QObject::tr("Grid generation"));
		progressDialog->setInfo(QObject::tr("Points: %L1\nCells: %L2 x %L3").arg( pointCount ).arg(width).arg(height));
		progressDialog->start();
		progressDialog->show();
		QCoreApplication::processEvents();
	}
	CCCoreLib::NormalizedProgress nProgress(progressDialog, pointCount);

	//vertical dimension
	assert(Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	//we always handle the colors (if any)
	hasColors = cloud->hasColors();

	//Array of pointers, each coresponding to a point in the cloud.
	//Pointers are used to chain together points belonging to the same cell.
	//'cloud->getPoint(n)' coresponds to 'pointRefList[n]'.
	std::vector<void*> pointRefList;
	try
	{
		pointRefList.resize(pointCount, nullptr);
	}
	catch (const std::bad_alloc)
	{
		ccLog::Error("Not enough memory");
		return false;
	}

	for (unsigned n = 0; n < pointCount; ++n)
	{
		//for each point
		const CCVector3* P = cloud->getPoint(n);

		//project it inside the grid
		CCVector2i cellPos = computeCellPos(*P, X, Y);

		//we skip points that fall outside of the grid!
		if (	cellPos.x < 0 || cellPos.x >= static_cast<int>(width)
			||	cellPos.y < 0 || cellPos.y >= static_cast<int>(height) )
		{
			if (!nProgress.oneStep())
			{
				//process cancelled by the user
				return false;
			}
			continue;
		}

		//update the cell statistics
		ccRasterCell& aCell = rows[cellPos.y][cellPos.x];
		
		//update linked list of point references
		if (aCell.nbPoints == 0)
		{
			//if first point in cell, set head and tail to this reference
			aCell.pointRefHead = pointRefList.data() + n;
			aCell.pointRefTail = pointRefList.data() + n;
		}
		else
		{
			//else point previous tail ref to this point, and reset tail
			*(aCell.pointRefTail) = pointRefList.data() + n;
			aCell.pointRefTail = pointRefList.data() + n;
		}

		//update the number of points in the cell
		++aCell.nbPoints;

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	//find maximum needed size for storing per-cell data
	unsigned maxCellPopuplation = 0;
	for (unsigned j = 0; j < height; ++j)
	{
		Row& row = rows[j];
		for (unsigned i = 0; i < width; ++i)
		{
			ccRasterCell& aCell = row[i];
			maxCellPopuplation = std::max(maxCellPopuplation, aCell.nbPoints);
		}
	}

	std::vector<IndexAndValue> cellPointIndexedHeight;
	std::vector<ScalarType> cellPointSF; 
	{
		try
		{
			cellPointIndexedHeight.resize(maxCellPopuplation);
			if (interpolateSF)
			{
				cellPointSF.resize(maxCellPopuplation);
			}
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory");
			return false;
		}
	}

	//now we can browse through all points belonging to each cell 
	for (unsigned j = 0; j < height; ++j)
	{
		Row& row = rows[j];
		for (unsigned i = 0; i < width; ++i)
		{
			ccRasterCell& aCell = row[i];
			if (aCell.nbPoints)
			{
				//Assemble a list of all points in this cell.
				//Start with first point in cell, and browse through the other points using the linked ref list, and collect point indexes
				void** pRef = aCell.pointRefHead;
				for (unsigned n = 0; n < aCell.nbPoints; ++n)
				{
					unsigned pointIndex = static_cast<unsigned>( pRef - pointRefList.data());
					const CCVector3* P = cloud->getPoint(pointIndex);
					cellPointIndexedHeight[n].index = pointIndex;
					cellPointIndexedHeight[n].val = P->u[Z];
					pRef = reinterpret_cast<void**> (*pRef);
				}

				auto cellPointIndexedHeightEnd = std::next(cellPointIndexedHeight.begin(), aCell.nbPoints);
				//sorting indexed points in cell based on height in ascending order
				std::sort(cellPointIndexedHeight.begin(), cellPointIndexedHeightEnd, [](const IndexAndValue& a, const IndexAndValue& b) { return a.val<b.val; } );

				//compute standard statistics on height values
				{
					//extract median value
					if (aCell.nbPoints % 2)
					{
						aCell.medianHeight = cellPointIndexedHeight[aCell.nbPoints / 2].val;
					}
					else
					{
						aCell.medianHeight = (cellPointIndexedHeight[(aCell.nbPoints / 2) - 1].val + cellPointIndexedHeight[aCell.nbPoints / 2].val) / 2;
					}

					//extract min/max value
					aCell.minHeight = cellPointIndexedHeight.front().val;
					aCell.maxHeight = cellPointIndexedHeight[aCell.nbPoints - 1].val;

					//calculate average value and std dev
					aCell.avgHeight = 0.0;
					double cellVariance = 0.0;
					for (unsigned n = 0; n < aCell.nbPoints; n++)
					{
						double h = cellPointIndexedHeight[n].val;
						aCell.avgHeight += h;
						cellVariance += h * h;
					}
					aCell.avgHeight /= aCell.nbPoints;
					cellVariance /= aCell.nbPoints;
					aCell.stdDevHeight = sqrt(std::abs(cellVariance - aCell.avgHeight*aCell.avgHeight));
				}
				
				//pick a point (index) that correspond to the selected 'height' value
				switch (projectionType)
				{
				case PROJ_MINIMUM_VALUE:
					aCell.h = aCell.minHeight;
					aCell.pointIndex = cellPointIndexedHeight.front().index;
					break;
				case PROJ_AVERAGE_VALUE:
					aCell.h = aCell.avgHeight;
					{
						//we choose the point which is the closest to the cell center (in 2D)
						CCVector2d C = computeCellCenter(i, j, X, Y);
						double minimumSquareDistToP = 0.0;
						for (unsigned n = 0; n < aCell.nbPoints; n++)
						{
							unsigned pointIndex = cellPointIndexedHeight[n].index;
							const CCVector3* P = cloud->getPoint(pointIndex);
							CCVector2d P2D(P->u[X], P->u[Y]);
							double squareDistToP = (C - P2D).norm2();
							if ((squareDistToP < minimumSquareDistToP) || (n == 0))
							{
								minimumSquareDistToP = squareDistToP;
								aCell.pointIndex = pointIndex;
							}
						}
					}
					break;
				case PROJ_MEDIAN_VALUE:
					aCell.h = aCell.medianHeight;
					aCell.pointIndex = cellPointIndexedHeight[aCell.nbPoints / 2].index;
					break;
				case PROJ_MAXIMUM_VALUE:
					aCell.h = aCell.maxHeight;
					aCell.pointIndex = cellPointIndexedHeight[aCell.nbPoints - 1].index;
					break;
				default:
					assert(false);
					break;
				}


				//if the cloud has RGB-colors 
				if (hasColors)
				{
					assert(cloud->hasColors());
					if (projectionType == PROJ_AVERAGE_VALUE)
					{
						//compute the average color
						aCell.color = CCVector3d(0,0,0);
						for (unsigned n = 0; n < aCell.nbPoints ;n++)
						{
							unsigned pointIndex = cellPointIndexedHeight[n].index;
							const ccColor::Rgb& col = cloud->getPointColor(pointIndex);
							aCell.color += CCVector3d(col.r, col.g, col.b);
						}
						aCell.color /= aCell.nbPoints;
					}
					else
					{  
						//pick color from selected index
						const ccColor::Rgb& col = cloud->getPointColor(aCell.pointIndex);
						aCell.color = CCVector3d(col.r, col.g, col.b);
					}
				}
				
				//if we should inteporlate the scalar fields
				if (interpolateSF)
				{
					assert(pc);
					//absolute position of the cell (e.g. in the 2D SF grid(s))
					int pos = j * static_cast<int>(width) + i;
					assert(pos < static_cast<int>(gridTotalSize));
					
					for (size_t k = 0; k < scalarFields.size(); ++k)
					{
						assert(!scalarFields[k].empty());
						CCCoreLib::ScalarField* sf = pc->getScalarField(static_cast<unsigned>(k));
						assert(sf && pos < scalarFields[k].size());
						//fill the vector of valid SF values for current cell 
						unsigned validPoints = 0;
						for (unsigned n = 0; n < aCell.nbPoints ;n++)
						{
							unsigned pointIndex = cellPointIndexedHeight[n].index;
							ScalarType sfValue = sf->getValue(pointIndex);
							if (CCCoreLib::ScalarField::ValidValue(sfValue))
							{
								cellPointSF[validPoints] = sfValue;
								++validPoints;
							}
						}
						
						if (validPoints == 0)
							continue;
						
						auto cellPointSFEnd = std::next(cellPointSF.begin(), validPoints);
						
						// Sort valid scalar values for cell
						std::sort(cellPointSF.begin(), cellPointSFEnd, [](ScalarType a, ScalarType b) { return a < b; } );

						switch (sfInterpolation)
						{
							case PROJ_MINIMUM_VALUE:
								scalarFields[k][pos] = cellPointSF.front();
								break;
							case PROJ_AVERAGE_VALUE:
								scalarFields[k][pos] = std::accumulate(cellPointSF.begin(), cellPointSFEnd, 0.0) / validPoints;
								break;
							case PROJ_MEDIAN_VALUE:
								if(validPoints%2)
								{
									scalarFields[k][pos] = cellPointSF[validPoints / 2];
								}
								else{
									scalarFields[k][pos] = (cellPointSF[(validPoints / 2) - 1] + cellPointSF[validPoints / 2]) / 2;
								}
								break;
							case PROJ_MAXIMUM_VALUE:
								scalarFields[k][pos] = cellPointSF[validPoints - 1];
								break;
							default:
								assert(false);
								break;
						}
					}
				}
			}
		}
	}

	//compute the number of non empty cells
	updateNonEmptyCellCount();

	//specific case: interpolate the empty cells
	if (doInterpolateEmptyCells)
	{
		interpolateEmptyCells(maxEdgeLength * maxEdgeLength);
	}

	//computation of the average and extreme height values in the grid
	updateCellStats();

	setValid(true);

	return true;
}

static void InterpolateOnBorder(const std::vector<uint8_t>& pointsOnBorder,
	const CCVector2i P[3],
	int i, int j,
	int coord,
	int dim,
	ccRasterCell& cell,
	ccRasterGrid& grid)
{
	uint8_t minIndex = pointsOnBorder[0];
	uint8_t maxIndex = pointsOnBorder[1];

	if (P[maxIndex][dim] < P[minIndex][dim])
		std::swap(minIndex, maxIndex);

	if (pointsOnBorder.size() == 3)
	{
		if (P[pointsOnBorder[2]][dim] < P[minIndex][dim])
			minIndex = pointsOnBorder[2];
		if (P[pointsOnBorder[2]][dim] > P[maxIndex][dim])
			maxIndex = pointsOnBorder[2];
	}

	if (P[minIndex][dim] <= coord && coord <= P[maxIndex][dim])
	{
		double d = P[maxIndex][dim] - P[minIndex][dim];
		if (d > 0)
		{
			//linear interpolation
			double relativePos = (coord - P[minIndex][dim]) / static_cast<double>(d);

			const ccRasterCell& A = grid.rows[P[minIndex].y][P[minIndex].x];
			const ccRasterCell& B = grid.rows[P[maxIndex].y][P[maxIndex].x];
			cell.h = (1.0 - relativePos) * A.h + relativePos * B.h;

			//interpolate color as well!
			if (grid.hasColors)
			{
				cell.color = (1.0 - relativePos) * A.color + relativePos * B.color;
			}

			//interpolate the SFs as well!
			for (auto &gridSF : grid.scalarFields)
			{
				assert(!gridSF.empty());

				double sfValA = gridSF[P[minIndex].x + P[minIndex].y * grid.width];
				double sfValB = gridSF[P[maxIndex].x + P[maxIndex].y * grid.width];
				assert(i + j * grid.width < gridSF.size());
				gridSF[i + j * grid.width] = (1.0 - relativePos) * sfValA + relativePos * sfValB;
			}

		}
		else //single point
		{
			const ccRasterCell& A = grid.rows[P[minIndex].y][P[minIndex].x];
			cell.h = A.h;

			if (grid.hasColors)
			{
				cell.color = A.color;
			}

			//interpolate the SFs as well!
			for (auto &gridSF : grid.scalarFields)
			{
				assert(!gridSF.empty());

				double sfValA = gridSF[P[minIndex].x + P[minIndex].y * grid.width];
				assert(i + j * grid.width < gridSF.size());
				gridSF[i + j * grid.width] = sfValA;
			}
		}
	}
}

bool ccRasterGrid::interpolateEmptyCells(double maxSquareEdgeLength)
{
	if (nonEmptyCellCount < 3)
	{
		ccLog::Warning("[Rasterize] Not enough non-empty cells for interpolation!");
		return false;
	}
	
	if (nonEmptyCellCount >= width * height)
	{
		//nothing to do
		return true;
	}

	std::vector<CCVector2> the2DPoints;
	try
	{
		the2DPoints.resize(nonEmptyCellCount);
	}
	catch (const std::bad_alloc&)
	{
		//out of memory
		ccLog::Warning("[Rasterize] Not enough memory to interpolate empty cells!");
		return false;
	}

	//fill 2D vector with non-empty cell indexes
	unsigned index = 0;
	for (unsigned j = 0; j < height; ++j)
	{
		const Row& row = rows[j];
		for (unsigned i = 0; i < width; ++i)
		{
			if (row[i].nbPoints)
			{
				//we only use the non-empty cells for interpolation
				the2DPoints[index++] = CCVector2(static_cast<PointCoordinateType>(i), static_cast<PointCoordinateType>(j));
			}
		}
	}
	assert(index == nonEmptyCellCount);

	//mesh the '2D' points
	CCCoreLib::Delaunay2dMesh delaunayMesh;
	std::string errorStr;
	if (!delaunayMesh.buildMesh(the2DPoints, CCCoreLib::Delaunay2dMesh::USE_ALL_POINTS, errorStr))
	{
		ccLog::Warning(QStringLiteral("[Rasterize] Empty cells interpolation failed. Could not compute the 2.5D mesh ('%1')")
			.arg(QString::fromStdString(errorStr)));
		return false;
	}

	//now we are going to 'project' all triangles on the grid
	delaunayMesh.placeIteratorAtBeginning();
	unsigned triNum = delaunayMesh.size();
	for (unsigned k = 0; k < triNum; ++k)
	{
		const CCCoreLib::VerticesIndexes* tsi = delaunayMesh.getNextTriangleVertIndexes();

		if (maxSquareEdgeLength > 0.0)
		{
			const CCVector2& A2D = the2DPoints[tsi->i[0]];
			const CCVector2& B2D = the2DPoints[tsi->i[1]];

			if ((B2D - A2D).norm2() > maxSquareEdgeLength)
			{
				continue;
			}

			const CCVector2& C2D = the2DPoints[tsi->i[2]];
			if (	(C2D - A2D).norm2() > maxSquareEdgeLength
				||	(C2D - B2D).norm2() > maxSquareEdgeLength)
			{
				continue;
			}
		}

		//get the triangle bounding box (in grid coordinates)
		CCVector2i P[3];
		int xMin = 0;
		int yMin = 0;
		int xMax = 0;
		int yMax = 0;
		//std::vector< uint8_t> onBottomBorder;
		std::vector< uint8_t> onTopBorder;
		//std::vector< uint8_t> onLeftBorder;
		std::vector< uint8_t> onRightBorder;
		{
			for (uint8_t k = 0; k < 3; ++k)
			{
				const CCVector2& P2D = the2DPoints[tsi->i[k]];
				P[k].x = static_cast<int>(P2D.x);
				P[k].y = static_cast<int>(P2D.y);

				//if (P[k].x == 0)
				//	onLeftBorder.push_back(k);
				if (static_cast<unsigned>(P[k].x + 1) == width)
					onRightBorder.push_back(k);
				//if (P[k].y == 0)
				//	onBottomBorder.push_back(k);
				if (static_cast<unsigned>(P[k].y + 1) == height)
					onTopBorder.push_back(k);
			}
			xMin = std::min(std::min(P[0].x, P[1].x), P[2].x);
			yMin = std::min(std::min(P[0].y, P[1].y), P[2].y);
			xMax = std::max(std::max(P[0].x, P[1].x), P[2].x);
			yMax = std::max(std::max(P[0].y, P[1].y), P[2].y);
		}
		
		//now scan the cells
		{
			//pre-computation for barycentric coordinates
			const double& valA = rows[P[0].y][P[0].x].h;
			const double& valB = rows[P[1].y][P[1].x].h;
			const double& valC = rows[P[2].y][P[2].x].h;

			int det = (P[1].y - P[2].y) * (P[0].x - P[2].x) - (P[1].x - P[2].x) * (P[0].y - P[2].y);

			for (int j = yMin; j <= yMax; ++j)
			{
				Row& row = rows[static_cast<unsigned>(j)];

				for (int i = xMin; i <= xMax; ++i)
				{
					//if the cell is empty
					if (!row[i].nbPoints)
					{
						//we test if it's included or not in the current triangle
						//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
						bool inside = false;
						if (det != 0)
						{
							for (int ti = 0; ti < 3; ++ti)
							{
								const CCVector2i& P1 = P[ti];
								const CCVector2i& P2 = P[(ti + 1) % 3];
								if ((P2.y <= j && j < P1.y) || (P1.y <= j && j < P2.y))
								{
									int t = (i - P2.x)*(P1.y - P2.y) - (P1.x - P2.x)*(j - P2.y);
									if (P1.y < P2.y)
										t = -t;
									if (t < 0)
										inside = !inside;
								}
							}
						}

						//can we interpolate?
						if (inside)
						{
							double l1 = ((P[1].y - P[2].y)*(i - P[2].x) - (P[1].x - P[2].x)*(j - P[2].y)) / static_cast<double>(det);
							double l2 = ((P[2].y - P[0].y)*(i - P[2].x) - (P[2].x - P[0].x)*(j - P[2].y)) / static_cast<double>(det);
							double l3 = 1.0 - l1 - l2;
							//if (l3 < 0.0 || l3 > 1.0)
							//{
							//	// shouldn't happen
							//	assert(false);
							//	inside = false;
							//}
							//else
							{
								row[i].h = l1 * valA + l2 * valB + l3 * valC;
								assert(std::isfinite(row[i].h));

								//interpolate color as well!
								if (hasColors)
								{
									const CCVector3d& colA = rows[P[0].y][P[0].x].color;
									const CCVector3d& colB = rows[P[1].y][P[1].x].color;
									const CCVector3d& colC = rows[P[2].y][P[2].x].color;
									row[i].color = l1 * colA + l2 * colB + l3 * colC;
								}

								//interpolate the SFs as well!
								for (auto &gridSF : scalarFields)
								{
									assert(!gridSF.empty());

									double sfValA = gridSF[P[0].x + P[0].y * width];
									double sfValB = gridSF[P[1].x + P[1].y * width];
									double sfValC = gridSF[P[2].x + P[2].y * width];
									assert(i + j * width < gridSF.size());
									gridSF[i + j * width] = l1 * sfValA + l2 * sfValB + l3 * sfValC;
								}
							}
						}

						// second test for the borders (only the top and right borders have this issue in fact)
						if (!inside)
						{
							/*if (i == 0 && onLeftBorder.size() > 1)
							{
								InterpolateOnBorder(onLeftBorder, P, i, j, j, 1, row[i], *this);
							}
							else */if (static_cast<unsigned>(i + 1) == width && onRightBorder.size() > 1)
							{
								InterpolateOnBorder(onRightBorder, P, i, j, j, 1, row[i], *this);
							}
							
							/*if (j == 0 && onBottomBorder.size() > 1)
							{
								InterpolateOnBorder(onBottomBorder, P, i, j, i, 0, row[i], *this);
							}
							else*/if (static_cast<unsigned>(j + 1) == height && onTopBorder.size() > 1)
							{
								InterpolateOnBorder(onTopBorder, P, i, j, i, 0, row[i], *this);
							}
						}
					}
				}
			}
		}
	}

	return true;
}

unsigned ccRasterGrid::updateNonEmptyCellCount()
{
	nonEmptyCellCount = 0;
	{
		for (unsigned i = 0; i < height; ++i)
			for (unsigned j = 0; j < width; ++j)
				if (rows[i][j].nbPoints)
					++nonEmptyCellCount;
	}
	return nonEmptyCellCount;
}

void ccRasterGrid::updateCellStats()
{
	minHeight = 0;
	maxHeight = 0;
	meanHeight = 0;
	validCellCount = 0;

	for (unsigned i = 0; i < height; ++i)
	{
		for (unsigned j = 0; j < width; ++j)
		{
			double h = rows[i][j].h;

			if (std::isfinite(h)) //valid height
			{
				if (validCellCount)
				{
					if (h < minHeight)
						minHeight = h;
					else if (h > maxHeight)
						maxHeight = h;

					meanHeight += h;
				}
				else
				{
					//first valid cell
					meanHeight = minHeight = maxHeight = h;
				}
				++validCellCount;
			}
		}
	}

	if (validCellCount)
	{
		meanHeight /= validCellCount;
	}
}


void ccRasterGrid::fillEmptyCells(	EmptyCellFillOption fillEmptyCellsStrategy,
									double customCellHeight/*=0*/)
{
	//fill empty cells (all but the 'INTERPOLATE' case)
	if (	fillEmptyCellsStrategy != LEAVE_EMPTY
		&&	fillEmptyCellsStrategy != INTERPOLATE)
	{
		double defaultHeight = std::numeric_limits<double>::quiet_NaN();
		switch (fillEmptyCellsStrategy)
		{
		case FILL_MINIMUM_HEIGHT:
			defaultHeight = minHeight;
			break;
		case FILL_MAXIMUM_HEIGHT:
			defaultHeight = maxHeight;
			break;
		case FILL_CUSTOM_HEIGHT:
			defaultHeight = customCellHeight;
			break;
		case FILL_AVERAGE_HEIGHT:
			defaultHeight = meanHeight;
			break;
		default:
			assert(false);
			break;
		}
		assert(defaultHeight != 0);

		for (unsigned i = 0; i < height; ++i)
		{
			for (unsigned j = 0; j < width; ++j)
			{
				if (!std::isfinite(rows[i][j].h)) //empty cell (NaN)
				{
					rows[i][j].h = defaultHeight;
				}
			}
		}
	}
}

ccPointCloud* ccRasterGrid::convertToCloud(	const std::vector<ExportableFields>& exportedFields,
											bool interpolateSF,
											bool interpolateColors,
											bool resampleInputCloudXY,
											bool resampleInputCloudZ,
											ccGenericPointCloud* inputCloud,
											unsigned char Z,
											const ccBBox& box,
											bool fillEmptyCells,
											double emptyCellsHeight,
											bool exportToOriginalCS) const
{
	if (Z > 2 || !box.isValid())
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	if (!isValid())
	{
		return nullptr;
	}

	unsigned pointsCount = (fillEmptyCells ? width * height : validCellCount);
	if (pointsCount == 0)
	{
		ccLog::Warning("[Rasterize] Empty grid!");
		return nullptr;
	}

	ccPointCloud* cloudGrid = nullptr;
	
	//if we 'resample' the input cloud, we actually resample it (one point in each cell)
	//and we may have to change some things afterwards (height, scalar fields, etc.)
	if (resampleInputCloudXY)
	{
		if (!inputCloud)
		{
			ccLog::Warning("[Rasterize] Internal error: no input clouds specified (for resampling)");
			assert(false);
			return nullptr;
		}
		CCCoreLib::ReferenceCloud refCloud(inputCloud);
		if (!refCloud.reserve(nonEmptyCellCount))
		{
			ccLog::Warning("[Rasterize] Not enough memory!");
			return nullptr;
		}

		for (unsigned j = 0; j < height; ++j)
		{
			for (unsigned i = 0; i < width; ++i)
			{
				const ccRasterCell& cell = rows[j][i];
				if (cell.nbPoints) //non empty cell
				{
					refCloud.addPointIndex(cell.pointIndex);
				}
			}
		}

		assert(refCloud.size() != 0);
		cloudGrid = inputCloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(inputCloud)->partialClone(&refCloud) : ccPointCloud::From(&refCloud, inputCloud);
		if (!cloudGrid)
		{
			ccLog::Error("[Rasterize] Not enough memory");
			return nullptr;
		}
		cloudGrid->setPointSize(0); //0 = default size (to avoid display issues)

		if (!resampleInputCloudZ)
		{
			//we have to use the grid height instead of the original point height!
			unsigned pointIndex = 0;
			for (unsigned j = 0; j < height; ++j)
			{
				for (unsigned i = 0; i < width; ++i)
				{
					const ccRasterCell& cell = rows[j][i];
					if (cell.nbPoints) //non empty cell
					{
						const_cast<CCVector3*>(cloudGrid->getPoint(pointIndex))->u[Z] = static_cast<PointCoordinateType>(cell.h);
						++pointIndex;
					}
				}
			}
		}

		if (!interpolateSF && !fillEmptyCells)
		{
			//DGM: we can't stop right away (even if we have already resampled the
			//original cloud, we may have to create additional points and/or scalar fields)
			//return cloudGrid;
		}
	}
	else
	{
		cloudGrid = new ccPointCloud("grid");
	}
	assert(cloudGrid);
	
	//shall we generate additional scalar fields?
	std::vector<CCCoreLib::ScalarField*> exportedSFs;
	if (!exportedFields.empty())
	{
		exportedSFs.resize(exportedFields.size(), nullptr);
		for (size_t i = 0; i < exportedFields.size(); ++i)
		{
			int sfIndex = -1;
			switch (exportedFields[i])
			{
			case PER_CELL_HEIGHT:
			case PER_CELL_COUNT:
			case PER_CELL_MIN_HEIGHT:
			case PER_CELL_MAX_HEIGHT:
			case PER_CELL_AVG_HEIGHT:
			case PER_CELL_MEDIAN_HEIGHT:
			case PER_CELL_HEIGHT_STD_DEV:
			case PER_CELL_HEIGHT_RANGE:
			{
				QString sfName = GetDefaultFieldName(exportedFields[i]);
				sfIndex = cloudGrid->getScalarFieldIndexByName(qPrintable(sfName));
				if (sfIndex >= 0)
				{
					ccLog::Warning(QString("[Rasterize] Scalar field '%1' already exists. It will be overwritten.").arg(sfName));
				}
				else
				{
					sfIndex = cloudGrid->addScalarField(qPrintable(sfName));
				}
			}
			break;
			default:
				assert(false);
				break;
			}
			
			if (sfIndex < 0)
			{
				ccLog::Warning("[Rasterize] Couldn't allocate scalar field(s)! Try to free some memory ...");
				break;
			}

			exportedSFs[i] = cloudGrid->getScalarField(sfIndex);
			assert(exportedSFs[i]);
		}
	}

	if (resampleInputCloudXY)
	{
		//if the cloud already has colors and we add the empty cells, we must also add (black) colors
		interpolateColors = (cloudGrid->hasColors() && fillEmptyCells);
	}
	else
	{
		//we need colors to interpolate them!
		interpolateColors &= hasColors;
	}

	//the resampled cloud already contains the points corresponding to 'filled' cells so we will only
	//need to add the empty ones (if requested)
	if (!resampleInputCloudXY || fillEmptyCells)
	{
		if (!cloudGrid->reserve(pointsCount))
		{
			ccLog::Warning("[Rasterize] Not enough memory!");
			delete cloudGrid;
			return nullptr;
		}

		if (interpolateColors && !cloudGrid->reserveTheRGBTable())
		{
			ccLog::Warning("[Rasterize] Not enough memory to interpolate colors!");
			cloudGrid->unallocateColors();
			interpolateColors = false;
		}
	}

	//horizontal dimensions
	const unsigned char X = (Z == 2 ? 0 : Z +1);
	const unsigned char Y = (X == 2 ? 0 : X +1);

	const unsigned char outX = (exportToOriginalCS ? X : 0);
	const unsigned char outY = (exportToOriginalCS ? Y : 1);
	const unsigned char outZ = (exportToOriginalCS ? Z : 2);

	//as the 'non empty cells points' are already in the cloud
	//we must take care of where we put the scalar fields values!
	unsigned nonEmptyCellIndex = 0;

	//we work with doubles as the grid step can be much smaller than the cloud coordinates!
	double Py = box.minCorner().u[Y]; //minCorner is the lower left cell CENTER

	for (unsigned j = 0; j < height; ++j)
	{
		const ccRasterCell* aCell = rows[j].data();
		double Px = box.minCorner().u[X]; //minCorner is the lower left cell CENTER
		
		for (unsigned i = 0; i < width; ++i, ++aCell)
		{
			if (std::isfinite(aCell->h)) //valid cell (could have been interpolated)
			{
				//if we haven't resampled the original cloud, we must add the point
				//corresponding to this non-empty cell
				if (!resampleInputCloudXY || aCell->nbPoints == 0)
				{
					CCVector3 Pf;
					Pf.u[outX] = static_cast<PointCoordinateType>(Px);
					Pf.u[outY] = static_cast<PointCoordinateType>(Py);
					Pf.u[outZ] = static_cast<PointCoordinateType>(aCell->h);

					assert(cloudGrid->size() < cloudGrid->capacity());
					cloudGrid->addPoint(Pf);

					if (interpolateColors)
					{
						ccColor::Rgb col(	static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.x)),
											static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.y)),
											static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.z)) );
						
						cloudGrid->addColor(col);
					}
				}

				//fill the associated SFs
				assert(!resampleInputCloudXY || inputCloud);
				assert(!resampleInputCloudXY || nonEmptyCellIndex < inputCloud->size()); //we can't be here if we have a fully resampled cloud! (resampleInputCloudXY implies that inputCloud is defined)
				assert(exportedSFs.size() == exportedFields.size());
				for (size_t k = 0; k < exportedSFs.size(); ++k)
				{
					CCCoreLib::ScalarField* sf = exportedSFs[k];
					if (!sf)
					{
						continue;
					}

					ScalarType sVal = CCCoreLib::NAN_VALUE;
					switch (exportedFields[k])
					{
					case PER_CELL_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->h);
						break;
					case PER_CELL_COUNT:
						sVal = static_cast<ScalarType>(aCell->nbPoints);
						break;
					case PER_CELL_MIN_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->minHeight);
						break;
					case PER_CELL_MAX_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->maxHeight);
						break;
					case PER_CELL_AVG_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->avgHeight);
						break;
					case PER_CELL_MEDIAN_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->medianHeight);
						break;
					case PER_CELL_HEIGHT_STD_DEV:
						sVal = static_cast<ScalarType>(aCell->stdDevHeight);
						break;
					case PER_CELL_HEIGHT_RANGE:
						sVal = static_cast<ScalarType>(aCell->maxHeight - aCell->minHeight);
						break;
					default:
						assert(false);
						break;
					}
					
					if (resampleInputCloudXY)
					{
						if (aCell->nbPoints != 0)
						{
							//previously existing point
							assert(nonEmptyCellIndex < inputCloud->size());
							assert(nonEmptyCellIndex < sf->size());
							sf->setValue(nonEmptyCellIndex, sVal);
						}
						else
						{
							//new point
							assert(sf->size() < sf->capacity());
							sf->addElement(sVal);
						}
					}
					else
					{
						//new point
						assert(sf->size() < sf->capacity());
						sf->addElement(sVal);
					}
				}

				if (aCell->nbPoints != 0)
				{
					++nonEmptyCellIndex;
				}
			}
			else if (fillEmptyCells) //empty cell
			{
				//even if we have resampled the original cloud, we must add the point
				//corresponding to this empty cell
				{
					CCVector3 Pf;
					Pf.u[outX] = static_cast<PointCoordinateType>(Px);
					Pf.u[outY] = static_cast<PointCoordinateType>(Py);
					Pf.u[outZ] = static_cast<PointCoordinateType>(emptyCellsHeight);
					
					assert(cloudGrid->size() < cloudGrid->capacity());
					cloudGrid->addPoint(Pf);

					if (interpolateColors)
					{
						cloudGrid->addColor(ccColor::black);
					}
				}

				assert(exportedSFs.size() == exportedFields.size());
				for (size_t k = 0; k < exportedSFs.size(); ++k)
				{
					CCCoreLib::ScalarField* sf = exportedSFs[k];
					if (!sf)
					{
						continue;
					}

					if (exportedFields[k] == PER_CELL_HEIGHT)
					{
						//we set the point height to the default height
						ScalarType s = static_cast<ScalarType>(emptyCellsHeight);
						assert(sf->size() < sf->capacity());
						sf->addElement(s);
					}
					else
					{
						assert(sf->size() < sf->capacity());
						sf->addElement(CCCoreLib::NAN_VALUE);
					}
				}
			}

			Px += gridStep;
		}

		Py += gridStep;
	}

	//finish the SFs initialization (if any)
	for (auto sf : exportedSFs)
	{
		if (sf)
		{
			sf->computeMinAndMax();
		}
	}

	//take care of former scalar fields
	if (!resampleInputCloudXY)
	{
		//do we need to interpolate the original SFs?
		if (interpolateSF && inputCloud && inputCloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(inputCloud);
			assert(scalarFields.size() == pc->getNumberOfScalarFields());
			
			for (size_t k = 0; k < scalarFields.size(); ++k)
			{
				assert(!scalarFields[k].empty());

				//the corresponding SF should exist on the input cloud
				ccScalarField* formerSf = static_cast<ccScalarField*>(pc->getScalarField(static_cast<int>(k)));
				assert(formerSf);

				//we try to create an equivalent SF on the output grid
				int sfIdx = cloudGrid->addScalarField(formerSf->getName());
				if (sfIdx < 0) //if we aren't lucky, the input cloud already had a SF with the same name
				{
					sfIdx = cloudGrid->addScalarField(qPrintable(QString(formerSf->getName()).append(".old")));
				}

				if (sfIdx < 0)
				{
					ccLog::Warning("[Rasterize] Failed to allocate a new scalar field for storing SF '%s' values! Try to free some memory ...", formerSf->getName());
				}
				else
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloudGrid->getScalarField(sfIdx));
					assert(sf);
					//set sf values
					unsigned n = 0;
					const ScalarType emptyCellSFValue = CCCoreLib::ScalarField::NaN();
					const double* _sfGrid = scalarFields[k].data();
					for (unsigned j = 0; j < height; ++j)
					{
						const ccRasterGrid::Row& row = rows[j];
						for (unsigned i = 0; i < width; ++i, ++_sfGrid)
						{
							if (std::isfinite(row[i].h)) //valid cell (could have been interpolated)
							{
								ScalarType s = static_cast<ScalarType>(*_sfGrid);
								sf->setValue(n++, s);
							}
							else if (fillEmptyCells)
							{
								sf->setValue(n++, emptyCellSFValue);
							}
						}
					}
					sf->computeMinAndMax();
					sf->importParametersFrom(formerSf);
					assert(sf->currentSize() == pointsCount);
				}
			}
		}
	}
	else //the cloud has already been resampled
	{
		//we simply add NAN values at the end of the SFs
		for (int k = 0; k < static_cast<int>(cloudGrid->getNumberOfScalarFields()); ++k)
		{
			CCCoreLib::ScalarField* sf = cloudGrid->getScalarField(k);
			sf->resizeSafe(cloudGrid->size(), true, CCCoreLib::NAN_VALUE);
		}
	}

	QString gridName = QString("raster(%1)").arg(gridStep);
	if (inputCloud)
	{
		gridName.prepend(inputCloud->getName() + QString("."));
	}
	cloudGrid->setName(gridName);

	return cloudGrid;
}
