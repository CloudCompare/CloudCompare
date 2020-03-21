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

//CCLib
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

	CCVector3d boxDiag = CCVector3d::fromArray(box.maxCorner().u) - CCVector3d::fromArray(box.minCorner().u);
	if (boxDiag.u[X] <= 0 || boxDiag.u[Y] <= 0)
	{
		ccLog::Warning("[ccRasterGrid::ComputeGridSize] Invalid cloud bounding box!");
		return false;
	}

	//DGM: we now use the 'PixelIsArea' convention (the height value is computed at the grid cell center)
	gridWidth = 1 + static_cast<unsigned>(boxDiag.u[X] / gridStep + 0.5);
	gridHeight = 1 + static_cast<unsigned>(boxDiag.u[Y] / gridStep + 0.5);

	return true;
}

void ccRasterGrid::clear()
{
	//reset
	width = height = 0;

	rows.resize(0);
	scalarFields.resize(0);

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

bool ccRasterGrid::fillWith(	ccGenericPointCloud* cloud,
								unsigned char Z,
								ProjectionType projectionType,
								bool interpolateEmptyCells,
								ProjectionType sfInterpolation/*=INVALID_PROJECTION_TYPE*/,
								ccProgressDialog* progressDialog/*=0*/)
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
	CCLib::NormalizedProgress nProgress(progressDialog, pointCount);

	//vertical dimension
	assert(Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;

	//we always handle the colors (if any)
	hasColors = cloud->hasColors();

	for (unsigned n = 0; n<pointCount; ++n)
	{
		//for each point
		const CCVector3* P = cloud->getPoint(n);

		//project it inside the grid
		CCVector3d relativePos = CCVector3d::fromArray(P->u) - minCorner;
		int i = static_cast<int>((relativePos.u[X] / gridStep + 0.5));
		int j = static_cast<int>((relativePos.u[Y] / gridStep + 0.5));

		//we skip points that fall outside of the grid!
		if (	i < 0 || i >= static_cast<int>(width)
			||	j < 0 || j >= static_cast<int>(height) )
		{
			if (!nProgress.oneStep())
			{
				//process cancelled by the user
				return false;
			}
			continue;
		}
		assert(i >= 0 && j >= 0);

		//update the cell statistics
		ccRasterCell& aCell = rows[j][i];
		if (aCell.nbPoints)
		{
			if (P->u[Z] < aCell.minHeight)
			{
				aCell.minHeight = P->u[Z];
				if (projectionType == PROJ_MINIMUM_VALUE)
				{
					//we keep track of the lowest point
					aCell.pointIndex = n;

					if (hasColors)
					{
						assert(cloud->hasColors());
						const ccColor::Rgb& col = cloud->getPointColor(n);
						aCell.color = CCVector3d(col.r, col.g, col.b);
					}
				}
			}
			else if (P->u[Z] > aCell.maxHeight)
			{
				aCell.maxHeight = P->u[Z];
				if (projectionType == PROJ_MAXIMUM_VALUE)
				{
					//we keep track of the highest point
					aCell.pointIndex = n;

					if (hasColors)
					{
						assert(cloud->hasColors());
						const ccColor::Rgb& col = cloud->getPointColor(n);
						aCell.color = CCVector3d(col.r, col.g, col.b);
					}
				}
			}

			if (projectionType == PROJ_AVERAGE_VALUE)
			{
				//we keep track of the point which is the closest to the cell center (in 2D)
				CCVector2d C((i + 0.5) * gridStep, (j + 0.5) * gridStep);
				const CCVector3* Q = cloud->getPoint(aCell.pointIndex); //former closest point
				CCVector3d relativePosQ = CCVector3d::fromArray(Q->u) - minCorner;

				double distToP = (C - CCVector2d(relativePos .u[X], relativePos .u[Y])).norm2();
				double distToQ = (C - CCVector2d(relativePosQ.u[X], relativePosQ.u[Y])).norm2();
				if (distToP < distToQ)
				{
					aCell.pointIndex = n;
				}

				if (hasColors)
				{
					assert(cloud->hasColors());
					const ccColor::Rgb& col = cloud->getPointColor(n);
					aCell.color += CCVector3d(col.r, col.g, col.b);
				}

			}
		}
		else
		{
			aCell.minHeight = aCell.maxHeight = P->u[Z];
			aCell.pointIndex = n;

			if (hasColors)
			{
				assert(cloud->hasColors());
				const ccColor::Rgb& col = cloud->getPointColor(n);
				aCell.color = CCVector3d(col.r, col.g, col.b);
			}
		}
		
		//sum the points heights
		double Pz = P->u[Z];
		aCell.avgHeight += Pz;
		aCell.stdDevHeight += Pz * Pz;

		//scalar fields
		if (interpolateSF)
		{
			assert(pc);

			//absolute position of the cell (e.g. in the 2D SF grid(s))
			int pos = j * static_cast<int>(width)+i;
			assert(pos < static_cast<int>(gridTotalSize));

			for (size_t k = 0; k < scalarFields.size(); ++k)
			{
				assert(!scalarFields[k].empty());

				CCLib::ScalarField* sf = pc->getScalarField(static_cast<unsigned>(k));
				assert(sf && pos < scalarFields[k].size());

				ScalarType sfValue = sf->getValue(n);

				if (ccScalarField::ValidValue(sfValue))
				{
					SF::value_type formerValue = scalarFields[k][pos];
					if (aCell.nbPoints && std::isfinite(formerValue))
					{
						switch (sfInterpolation)
						{
						case PROJ_MINIMUM_VALUE:
							// keep the minimum value
							scalarFields[k][pos] = std::min<SF::value_type>(formerValue, sfValue);
							break;
						case PROJ_AVERAGE_VALUE:
							//we sum all values (we will divide them later)
							scalarFields[k][pos] += sfValue;
							break;
						case PROJ_MAXIMUM_VALUE:
							// keep the maximum value
							scalarFields[k][pos] = std::max<SF::value_type>(formerValue, sfValue);
							break;
						default:
							assert(false);
							break;
						}
					}
					else
					{
						//for the first (valid) point, we simply have to store its SF value (in any case)
						scalarFields[k][pos] = sfValue;
					}
				}
			}
		}

		//update the number of points in the cell
		++aCell.nbPoints;

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	//update SF grids for 'average' cases
	if (sfInterpolation == PROJ_AVERAGE_VALUE)
	{
		for (auto &scalarField : scalarFields)
		{
			assert(!scalarField.empty());

			double* _gridSF = scalarField.data();
			for (unsigned j = 0; j < height; ++j)
			{
				Row& row = rows[j];
				for (unsigned i = 0; i < width; ++i, ++_gridSF)
				{
					if (row[i].nbPoints > 1)
					{
						if (std::isfinite(*_gridSF)) //valid SF value
						{
							*_gridSF /= row[i].nbPoints;
						}
					}
				}
			}
		}
	}

	//update the main grid (average height and std.dev. computation + current 'height' value)
	{
		for (unsigned j = 0; j < height; ++j)
		{
			Row& row = rows[j];
			for (unsigned i = 0; i < width; ++i)
			{
				ccRasterCell& cell = row[i];
				if (cell.nbPoints > 1)
				{
					cell.avgHeight /= cell.nbPoints;
					cell.stdDevHeight = sqrt(fabs(cell.stdDevHeight / cell.nbPoints - cell.avgHeight*cell.avgHeight));
					if (hasColors && projectionType == PROJ_AVERAGE_VALUE)
					{
						cell.color /= cell.nbPoints;
					}
				}
				else
				{
					cell.stdDevHeight = 0;
				}

				if (cell.nbPoints != 0)
				{
					//set the right 'height' value
					switch (projectionType)
					{
					case PROJ_MINIMUM_VALUE:
						cell.h = cell.minHeight;
						break;
					case PROJ_AVERAGE_VALUE:
						cell.h = cell.avgHeight;
						break;
					case PROJ_MAXIMUM_VALUE:
						cell.h = cell.maxHeight;
						break;
					default:
						assert(false);
						break;
					}
				}
			}
		}
	}

	//compute the number of non empty cells
	nonEmptyCellCount = 0;
	{
		for (unsigned i = 0; i < height; ++i)
			for (unsigned j = 0; j < width; ++j)
				if (rows[i][j].nbPoints)
					++nonEmptyCellCount;
	}

	//specific case: interpolate the empty cells
	if (interpolateEmptyCells)
	{
		std::vector<CCVector2> the2DPoints;
		if (nonEmptyCellCount < 3)
		{
			ccLog::Warning("[Rasterize] Not enough non-empty cells for interpolation!");
		}
		else if (nonEmptyCellCount < width * height) //otherwise it's useless!
		{
			try
			{
				the2DPoints.resize(nonEmptyCellCount);
			}
			catch (const std::bad_alloc&)
			{
				//out of memory
				ccLog::Warning("[Rasterize] Not enough memory to interpolate empty cells!");
			}
		}

		//fill 2D vector with non-empty cell indexes
		if (!the2DPoints.empty())
		{
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
			CCLib::Delaunay2dMesh delaunayMesh;
			char errorStr[1024];
			if (delaunayMesh.buildMesh(the2DPoints, 0, errorStr))
			{
				unsigned triNum = delaunayMesh.size();
				//now we are going to 'project' all triangles on the grid
				delaunayMesh.placeIteratorAtBeginning();
				for (unsigned k = 0; k < triNum; ++k)
				{
					const CCLib::VerticesIndexes* tsi = delaunayMesh.getNextTriangleVertIndexes();
					//get the triangle bounding box (in grid coordinates)
					int P[3][2];
					int xMin = 0;
					int yMin = 0;
					int xMax = 0;
					int yMax = 0;
					{
						for (unsigned j = 0; j < 3; ++j)
						{
							const CCVector2& P2D = the2DPoints[tsi->i[j]];
							P[j][0] = static_cast<int>(P2D.x);
							P[j][1] = static_cast<int>(P2D.y);
						}
						xMin = std::min(std::min(P[0][0], P[1][0]), P[2][0]);
						yMin = std::min(std::min(P[0][1], P[1][1]), P[2][1]);
						xMax = std::max(std::max(P[0][0], P[1][0]), P[2][0]);
						yMax = std::max(std::max(P[0][1], P[1][1]), P[2][1]);
					}
					//now scan the cells
					{
						//pre-computation for barycentric coordinates
						const double& valA = rows[ P[0][1] ][ P[0][0] ].h;
						const double& valB = rows[ P[1][1] ][ P[1][0] ].h;
						const double& valC = rows[ P[2][1] ][ P[2][0] ].h;

						double det = static_cast<double>((P[1][1] - P[2][1])*(P[0][0] - P[2][0]) + (P[2][0] - P[1][0])*(P[0][1] - P[2][1]));

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
									for (int ti = 0; ti < 3; ++ti)
									{
										const int* P1 = P[ti];
										const int* P2 = P[(ti + 1) % 3];
										if ((P2[1] <= j &&j < P1[1]) || (P1[1] <= j && j < P2[1]))
										{
											int t = (i - P2[0])*(P1[1] - P2[1]) - (P1[0] - P2[0])*(j - P2[1]);
											if (P1[1] < P2[1])
												t = -t;
											if (t < 0)
												inside = !inside;
										}
									}
									//can we interpolate?
									if (inside)
									{
										double l1 = ((P[1][1] - P[2][1])*(i - P[2][0]) + (P[2][0] - P[1][0])*(j - P[2][1])) / det;
										double l2 = ((P[2][1] - P[0][1])*(i - P[2][0]) + (P[0][0] - P[2][0])*(j - P[2][1])) / det;
										double l3 = 1.0 - l1 - l2;

										row[i].h = l1 * valA + l2 * valB + l3 * valC;
										assert(std::isfinite(row[i].h));

										//interpolate color as well!
										if (hasColors)
										{
											const CCVector3d& colA = rows[P[0][1]][P[0][0]].color;
											const CCVector3d& colB = rows[P[1][1]][P[1][0]].color;
											const CCVector3d& colC = rows[P[2][1]][P[2][0]].color;
											row[i].color = l1 * colA + l2 * colB + l3 * colC;
										}

										//interpolate the SFs as well!
										for (auto &gridSF : scalarFields)
										{
											assert(!gridSF.empty());

											const double& sfValA = gridSF[P[0][0] + P[0][1] * width];
											const double& sfValB = gridSF[P[1][0] + P[1][1] * width];
											const double& sfValC = gridSF[P[2][0] + P[2][1] * width];
											assert(i + j * width < gridSF.size());
											gridSF[i + j * width] = l1 * sfValA + l2 * sfValB + l3 * sfValC;
										}
									}
								}
							}
						}
					}
				}
			}
			else
			{
				ccLog::Warning(QString("[Rasterize] Empty cells interpolation failed: Triangle lib. said '%1'").arg(errorStr));
			}
		}
	}

	//computation of the average and extreme height values in the grid
	{
		minHeight = 0;
		maxHeight = 0;
		meanHeight = 0;
		validCellCount = 0;

		for (unsigned i=0; i<height; ++i)
		{
			for (unsigned j=0; j<width; ++j)
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

	setValid(true);

	return true;
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
	//and we may have to change some things aftewards (height, scalar fields, etc.)
	if (resampleInputCloudXY)
	{
		if (!inputCloud)
		{
			ccLog::Warning("[Rasterize] Internal error: no input clouds specified (for resampling)");
			assert(false);
			return nullptr;
		}
		CCLib::ReferenceCloud refCloud(inputCloud);
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
	std::vector<CCLib::ScalarField*> exportedSFs;
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
	double Py = box.minCorner().u[Y]/* + gridStep / 2*/;

	for (unsigned j = 0; j < height; ++j)
	{
		const ccRasterCell* aCell = rows[j].data();
		double Px = box.minCorner().u[X]/* + gridStep / 2*/;
		
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
					CCLib::ScalarField* sf = exportedSFs[k];
					if (!sf)
					{
						continue;
					}

					ScalarType sVal = NAN_VALUE;
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
					CCLib::ScalarField* sf = exportedSFs[k];
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
						sf->addElement(NAN_VALUE);
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
					const ScalarType emptyCellSFValue = CCLib::ScalarField::NaN();
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
			CCLib::ScalarField* sf = cloudGrid->getScalarField(k);
			sf->resizeSafe(cloudGrid->size(), true, NAN_VALUE);
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
