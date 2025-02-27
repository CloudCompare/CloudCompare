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
#include <ParallelSort.h>

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
		insert(ccRasterGrid::PER_CELL_VALUE,              "Cell height values");
		insert(ccRasterGrid::PER_CELL_COUNT,              "population");
		insert(ccRasterGrid::PER_CELL_MIN_VALUE,          "min");
		insert(ccRasterGrid::PER_CELL_MAX_VALUE,          "max");
		insert(ccRasterGrid::PER_CELL_AVG_VALUE,          "average");
		insert(ccRasterGrid::PER_CELL_VALUE_STD_DEV,      "std. dev.");
		insert(ccRasterGrid::PER_CELL_VALUE_RANGE,        "range");
		insert(ccRasterGrid::PER_CELL_MEDIAN_VALUE,       "median");
		insert(ccRasterGrid::PER_CELL_UNIQUE_COUNT_VALUE, "unique");
		insert(ccRasterGrid::PER_CELL_PERCENTILE_VALUE,   "percentile");
	}
};
static DefaultFieldNames s_defaultFieldNames;

QString ccRasterGrid::GetDefaultFieldName(ExportableFields field)
{
	assert(s_defaultFieldNames.contains(field));
	return s_defaultFieldNames[field];
}

void ccRasterCell::getPointIndexes(std::vector<unsigned>& indexes, const std::vector<void*>& pointRefList) const
{
	indexes.clear();
	// Assemble a list of all point indexes in this cell
	// Start with first point in cell, and browse through every point using the linked ref list, and collect point indexes
	void** pRef = pointRefHead;
	for (unsigned n = 0; n < nbPoints; ++n)
	{
		unsigned pointIndex = static_cast<unsigned>(pRef - pointRefList.data());
		indexes.push_back(pointIndex);
		pRef = reinterpret_cast<void**> (*pRef);
	}
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

ccRasterGrid::InterpolationType ccRasterGrid::InterpolationTypeFromEmptyCellFillOption(EmptyCellFillOption option)
{
	switch (option)
	{
	case INTERPOLATE_DELAUNAY:
		return InterpolationType::DELAUNAY;
	case KRIGING:
		return InterpolationType::KRIGING;
	default:
		break;
	}

	return InterpolationType::NONE;
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
								InterpolationType emptyCellsInterpolation/*=InterpolationType::NONE*/,
								void* interpolationParams/*=nullptr*/,
								ProjectionType sfProjectionType/*=INVALID_PROJECTION_TYPE*/,
								ccProgressDialog* progressDialog/*=nullptr*/,
								int zStdDevSfIndex/*=-1*/)
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

	//do we need to project scalar fields?
	bool projectSFs = (sfProjectionType != INVALID_PROJECTION_TYPE);
	if (projectSFs)
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
			projectSFs = false;
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

	std::vector<IndexAndValue> cellPointIndexedHeight;
	std::vector<ScalarType> cellInvVarianceValues;
	try			
	{
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

		cellPointIndexedHeight.resize(maxCellPopuplation);
		if (projectionType == PROJ_INVERSE_VAR_VALUE)
		{
			cellInvVarianceValues.resize(maxCellPopuplation);
		}
	}
	catch (const std::bad_alloc&)
	{
		//out of memory
		ccLog::Warning("Not enough memory");
		return false;
	}
   
    // Find the right 'std. dev.' SF if inverse variance is being used
    CCCoreLib::ScalarField* zStdDevSF = nullptr;
	if (projectionType == PROJ_INVERSE_VAR_VALUE || sfProjectionType == PROJ_INVERSE_VAR_VALUE)
	{
		if (zStdDevSfIndex >= 0)
		{
			assert(pc);
			zStdDevSF = pc->getScalarField(zStdDevSfIndex);
		}

		if (!zStdDevSF)
		{
			ccLog::Warning("Internal error: invalid std. dev. SF (index)");
			return false;
		}
	}

	std::vector<ScalarType> sfValues; // common vector used to sort SF values in each cell

	//now we can browse through all points belonging to each cell 
	for (unsigned j = 0; j < height; ++j)
	{
		Row& row = rows[j];
		for (unsigned i = 0; i < width; ++i)
		{
			ccRasterCell& aCell = row[i];
			double cellAvgHeight = 0.0;
			double cellStdDevHeight = 0.0;
			double cellModelStdDevHeight = std::numeric_limits<double>::quiet_NaN(); // for inv. var. projection mode only

			if (aCell.nbPoints)
			{
				//Assemble a list of all points in this cell.
				//Start with first point in cell, and browse through the other points using the linked ref list, and collect point indexes
				void** pRef = aCell.pointRefHead;
				for (unsigned n = 0; n < aCell.nbPoints; ++n)
				{
					unsigned pointIndex = static_cast<unsigned>(pRef - pointRefList.data());
					const CCVector3* P = cloud->getPoint(pointIndex);
					cellPointIndexedHeight[n].index = pointIndex;
					cellPointIndexedHeight[n].val = P->u[Z];
					pRef = reinterpret_cast<void**> (*pRef);
				}

				auto cellPointIndexedHeightEnd = std::next(cellPointIndexedHeight.begin(), aCell.nbPoints);
				//sorting indexed points in cell based on height in ascending order
				ParallelSort(cellPointIndexedHeight.begin(), cellPointIndexedHeightEnd, [](const IndexAndValue& a, const IndexAndValue& b) { return a.val < b.val; });

				//compute standard statistics on height values

				//extract min/max value
				aCell.minHeight = cellPointIndexedHeight.front().val;
				aCell.maxHeight = cellPointIndexedHeight[aCell.nbPoints - 1].val;

				if (projectionType != PROJ_INVERSE_VAR_VALUE) 
				{
					//calculate average value and std dev
					cellAvgHeight = 0.0;
					double cellSquareSum = 0.0;
					for (unsigned n = 0; n < aCell.nbPoints; n++)
					{
						double h = cellPointIndexedHeight[n].val;
						cellAvgHeight += h;
						cellSquareSum += h * h;
					}
					cellAvgHeight /= aCell.nbPoints;
					cellStdDevHeight = sqrt(std::max(0.0, cellSquareSum / aCell.nbPoints - cellAvgHeight * cellAvgHeight));
				}
				else // inverse variance projection mode
				{
					assert(zStdDevSF);
					// Calculate weighted average
					double sumInverseVariance = 0.0;
					double weightedSum = 0.0;
					double weightedSquareSum = 0.0;
					for (unsigned n = 0; n < aCell.nbPoints; ++n)
					{
						// Compute inverse variance for all points in the current cell 
						ScalarType stdDev = zStdDevSF->getValue(cellPointIndexedHeight[n].index);
						if (ccScalarField::ValidValue(stdDev) && CCCoreLib::GreaterThanEpsilon(stdDev))
						{
							double invVar = 1.0 / (static_cast<double>(stdDev) * stdDev);
							weightedSum += invVar * cellPointIndexedHeight[n].val;
							weightedSquareSum += invVar * cellPointIndexedHeight[n].val * cellPointIndexedHeight[n].val;
							sumInverseVariance += invVar;

							cellInvVarianceValues[n] = static_cast<ScalarType>(invVar);
						}
						else
						{
							cellInvVarianceValues[n] = CCCoreLib::NAN_VALUE;
						}
					}
					
					if (CCCoreLib::GreaterThanEpsilon(sumInverseVariance))
					{
						cellAvgHeight = weightedSum / sumInverseVariance;
						cellStdDevHeight = std::sqrt(std::abs(weightedSquareSum / sumInverseVariance - cellAvgHeight * cellAvgHeight));
						cellModelStdDevHeight = std::sqrt(1.0 / sumInverseVariance);
					}
					else
					{
						// we can't compute these values if the weight is null (= no valid SF value)
						cellAvgHeight = cellStdDevHeight = cellModelStdDevHeight = std::numeric_limits<double>::quiet_NaN();
					}
				}
				
				//pick a point (index) that correspond to the selected 'height' value
				switch (projectionType)
				{
				case PROJ_MINIMUM_VALUE:
					aCell.h = aCell.minHeight;
					aCell.nearestPointIndex = cellPointIndexedHeight.front().index;
					break;
				case PROJ_AVERAGE_VALUE:
				case PROJ_INVERSE_VAR_VALUE:
					aCell.h = cellAvgHeight;
					if (std::isfinite(aCell.h))
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
								aCell.nearestPointIndex = pointIndex;
							}
						}
					}
					break;
				case PROJ_MEDIAN_VALUE:
				{
					//extract median value
					unsigned indexMid = aCell.nbPoints / 2;
					if (aCell.nbPoints % 2) // odd value
					{
						aCell.h = cellPointIndexedHeight[indexMid].val;
					}
					else
					{
						aCell.h = (cellPointIndexedHeight[indexMid - 1].val + cellPointIndexedHeight[indexMid].val) / 2;
					}
					aCell.nearestPointIndex = cellPointIndexedHeight[indexMid].index;
				}
				break;
				case PROJ_MAXIMUM_VALUE:
					aCell.h = aCell.maxHeight;
					aCell.nearestPointIndex = cellPointIndexedHeight[aCell.nbPoints - 1].index;
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
						aCell.color = CCVector3d(0, 0, 0);
						for (unsigned n = 0; n < aCell.nbPoints; n++)
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
						const ccColor::Rgb& col = cloud->getPointColor(aCell.nearestPointIndex);
						aCell.color = CCVector3d(col.r, col.g, col.b);
					}
				}
				
				//if we should project the scalar fields
				if (projectSFs)
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

						switch (sfProjectionType)
						{
						case PROJ_MINIMUM_VALUE:
						{
							ScalarType minValue = CCCoreLib::NAN_VALUE;
							for (unsigned n = 0; n < aCell.nbPoints; n++)
							{
								unsigned pointIndex = cellPointIndexedHeight[n].index;
								ScalarType value = sf->getValue(pointIndex);
								if (CCCoreLib::ScalarField::ValidValue(value))
								{
									if (std::isnan(minValue) || minValue > value)
									{
										minValue = value;
									}
								}
							}
							scalarFields[k][pos] = minValue;
						}
						break;

						case PROJ_MEDIAN_VALUE:
						{
							sfValues.clear();
							sfValues.reserve(aCell.nbPoints);
							for (unsigned n = 0; n < aCell.nbPoints; n++)
							{
								unsigned pointIndex = cellPointIndexedHeight[n].index;
								ScalarType value = sf->getValue(pointIndex);
								if (CCCoreLib::ScalarField::ValidValue(value))
								{
									sfValues.push_back(value);
								}
							}
							if (sfValues.size() > 1)
							{
								ParallelSort(sfValues.begin(), sfValues.end());
								size_t midIndex = sfValues.size() / 2;
								if (sfValues.size() % 2) // odd number
								{
									scalarFields[k][pos] = sfValues[midIndex];
								}
								else
								{
									scalarFields[k][pos] = static_cast<ScalarType>((static_cast<double>(sfValues[midIndex - 1]) + sfValues[midIndex]) / 2);
								}
							}
							else if (sfValues.size() == 1)
							{
								scalarFields[k][pos] = sfValues[0];
							}
							else
							{
								scalarFields[k][pos] = CCCoreLib::NAN_VALUE;
							}
						}
						break;

						case PROJ_MAXIMUM_VALUE:
						{
							ScalarType maxValue = CCCoreLib::NAN_VALUE;
							for (unsigned n = 0; n < aCell.nbPoints; n++)
							{
								unsigned pointIndex = cellPointIndexedHeight[n].index;
								ScalarType value = sf->getValue(pointIndex);
								if (CCCoreLib::ScalarField::ValidValue(value))
								{
									if (std::isnan(maxValue) || maxValue < value)
									{
										maxValue = value;
									}
								}
							}
							scalarFields[k][pos] = maxValue;
						}
						break;

						case PROJ_AVERAGE_VALUE:
						{
							//for average, we do a simple average of unsorted SF-values in cell
							double scalarFieldWeightedSum = 0.0;
							unsigned validPointCount = 0;
							for (unsigned n = 0; n < aCell.nbPoints; n++)
							{
								unsigned pointIndex = cellPointIndexedHeight[n].index;
								ScalarType value = sf->getValue(pointIndex);
								if (CCCoreLib::ScalarField::ValidValue(value))
								{
									scalarFieldWeightedSum += value;
									++validPointCount;
								}
							}
							scalarFields[k][pos] = validPointCount != 0 ? scalarFieldWeightedSum / validPointCount : std::numeric_limits<double>::quiet_NaN();
						}
						break;

						case PROJ_INVERSE_VAR_VALUE:
							//inverse variance projection mode: weighted average with weights of 1/var
							if (projectionType == PROJ_INVERSE_VAR_VALUE && k == zStdDevSfIndex)
							{
								//Special case for the 'std deviation' scalar field, output layer should
								//just be filled with the updated model standard deviation.
								scalarFields[k][pos] = cellModelStdDevHeight;
							}
							else
							{
								assert(zStdDevSF);
								double scalarFieldWeightedSum = 0.0;
								double scalarFieldWeightSum = 0.0;
								for (unsigned n = 0; n < aCell.nbPoints; n++)
								{
									unsigned pointIndex = cellPointIndexedHeight[n].index;
									ScalarType stdDev = zStdDevSF->getValue(pointIndex);
									if (ccScalarField::ValidValue(stdDev) && CCCoreLib::GreaterThanEpsilon(stdDev))
									{
										ScalarType value = sf->getValue(pointIndex);
										if (ccScalarField::ValidValue(value))
										{
											ScalarType weight = 1.0 / (stdDev*stdDev);
											scalarFieldWeightedSum += static_cast<double>(weight) * value;
											scalarFieldWeightSum += weight;
										}
									}
								}

								scalarFields[k][pos] = CCCoreLib::GreaterThanEpsilon(scalarFieldWeightSum) ? scalarFieldWeightedSum / scalarFieldWeightSum : std::numeric_limits<double>::quiet_NaN();

							}
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
	switch (emptyCellsInterpolation)
	{
	case InterpolationType::NONE:
		// do nothing
		break;

	case InterpolationType::DELAUNAY:
	{
		DelaunayInterpolationParams* const params = reinterpret_cast<DelaunayInterpolationParams* const>(interpolationParams);
		if (params)
		{
			interpolateEmptyCells(params->maxEdgeLength * params->maxEdgeLength);
		}
		else
		{
			ccLog::Error("[Rasterize] Internal error: Delauny interpolation parameters are not set");
		}
	}
	break;

	case InterpolationType::KRIGING:
	{
		KrigingParams* krigingParams = reinterpret_cast<KrigingParams*>(interpolationParams);
		if (krigingParams)
		{
			fillGridCellsWithKriging(Z, krigingParams->kNN, krigingParams->params, !krigingParams->autoGuess, progressDialog);
		}
		else
		{
			ccLog::Error("[Rasterize] Internal error: Krigin interpolation parameters are not set");
		}
	}
	break;

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

	if (maxSquareEdgeLength > 0.0)
	{
		// we have to scale the maxSquareEdgeLength parameters as we will consider now 'cells' and not real units
		maxSquareEdgeLength /= (gridStep * gridStep);
	}

	//now we are going to 'project' all triangles on the grid
	delaunayMesh.placeIteratorAtBeginning();
	unsigned triNum = delaunayMesh.size();
#ifdef _DEBUG
	size_t interpolatedCells = 0;
	size_t toolLongEdgeCount = 0;
#endif
	for (unsigned k = 0; k < triNum; ++k)
	{
		const CCCoreLib::VerticesIndexes* tsi = delaunayMesh.getNextTriangleVertIndexes();

		if (maxSquareEdgeLength > 0.0)
		{
			const CCVector2& A2D = the2DPoints[tsi->i[0]];
			const CCVector2& B2D = the2DPoints[tsi->i[1]];

			if ((B2D - A2D).norm2() > maxSquareEdgeLength)
			{
#ifdef _DEBUG
				++toolLongEdgeCount;
#endif
				continue;
			}

			const CCVector2& C2D = the2DPoints[tsi->i[2]];
			if (	(C2D - A2D).norm2() > maxSquareEdgeLength
				||	(C2D - B2D).norm2() > maxSquareEdgeLength)
			{
#ifdef _DEBUG
				++toolLongEdgeCount;
#endif
				continue;
			}
		}

		//get the triangle bounding box (in grid coordinates)
		CCVector2i P[3];
		int xMin = 0;
		int yMin = 0;
		int xMax = 0;
		int yMax = 0;
		//std::vector<uint8_t> onBottomBorder;
		std::vector<uint8_t> onTopBorder;
		//std::vector<uint8_t> onLeftBorder;
		std::vector<uint8_t> onRightBorder;
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
					if (!row[i].nbPoints && !std::isfinite(row[i].h))
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

							row[i].h = l1 * valA + l2 * valB + l3 * valC;
							//assert(std::isfinite(row[i].h)); //it can happen with the inv. var. projection mode
#ifdef _DEBUG
							++interpolatedCells;
#endif

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
						else // second test for the borders (only the top and right borders have this issue in fact)
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

bool ccRasterGrid::fillGridCellsWithKriging(unsigned char Z,
											int knn,
											Kriging::KrigeParams& krigeParams,
											bool useInputParams,
											ccProgressDialog* progressDialog/*=nullptr*/)
{
	if (Z > 2)
	{
		ccLog::Error("Internal error: invalid Z dimension");
		return false;
	}
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	if (nonEmptyCellCount == 0)
	{
		ccLog::Warning("Not enough non-empty cells to perform Kriging");
		return false;
	}

	// Kriging structure
	std::vector<DataPoint> dataPoints;
	try
	{
		dataPoints.reserve(nonEmptyCellCount);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(QObject::tr("Kriging: not enough memory"));
		return false;
	}

	if (progressDialog)
	{
		progressDialog->setMethodTitle(QObject::tr("Kriging").toStdString().c_str());
		progressDialog->setInfo(QObject::tr("Non-empty cells: %1\nGrid: %2 x %3").arg(nonEmptyCellCount).arg(width).arg(height));
		progressDialog->start();
		progressDialog->show();
		QCoreApplication::processEvents();
	}

	// use non-empty cells
	{
		for (unsigned j = 0; j < height; ++j)
		{
			const Row& row = rows[j];
			CCVector2d point(0.0, (j + 0.5) * gridStep);

			for (unsigned i = 0; i < width; ++i)
			{
				point.x = (i + 0.5) * gridStep;

				const ccRasterCell& cell = row[i];
				if (cell.nbPoints)
				{
					dataPoints.push_back(DataPoint(point.x, point.y, cell.h));
				}
			}
		}
	}

	RasterParameters rasterParams(CCVector2d(0, 0), gridStep, width, height);

	unsigned stepCount = 1;
	stepCount += static_cast<unsigned>(scalarFields.size());
	if (hasColors)
		stepCount += 3;

	CCCoreLib::NormalizedProgress nProgress(progressDialog, static_cast<unsigned>(nonEmptyCellCount * stepCount));

	Kriging kriging(dataPoints, rasterParams);
	knn = std::min(knn, static_cast<int>(nonEmptyCellCount - 1));

	auto* context = kriging.createOrdinaryKrigeContext(knn);
	if (!context)
	{
		ccLog::Error(QObject::tr("Failed to initialize the Kriging algorithm"));
		return false;
	}

	// process the altitudes first
	{
		if (!useInputParams)
		{
			// compute default parameters
			Kriging::Model model = krigeParams.model;
			krigeParams = kriging.computeDefaultParameters();
			if (model != Kriging::Invalid)
			{
				// restore the input model
				krigeParams.model = model;
			}
		}

		for (unsigned j = 0; j < height; ++j)
		{
			Row& row = rows[j];

			for (unsigned i = 0; i < width; ++i)
			{
				ccRasterCell& cell = row[i];
				cell.h = kriging.ordinaryKrigeSingleCell(krigeParams, i, j, context);

				if (!nProgress.oneStep())
				{
					//process cancelled by user
					kriging.releaseOrdinaryKrigeContext(context);
					return false;
				}
			}
		}
	}

	// then process the scalar values (if any)
	for (size_t sfIndex = 0; sfIndex < scalarFields.size(); ++sfIndex)
	{
		SF& sf = scalarFields[sfIndex];

		// update the kriging value
		{
			size_t index = 0;
			for (unsigned j = 0; j < height; ++j)
			{
				const Row& row = rows[j];
				for (unsigned i = 0; i < width; ++i)
				{
					const ccRasterCell& cell = row[i];
					if (cell.nbPoints)
					{
						dataPoints[index++].value = sf[i + j * width];
					}
				}
			}
			assert(index == nonEmptyCellCount);
		}

		// compute default parameters
		Kriging::KrigeParams sfKrigeParams = kriging.computeDefaultParameters();
		// use the same model as for the altitudes
		if (krigeParams.model != Kriging::Invalid)
		{
			sfKrigeParams.model = krigeParams.model;
		}

		for (unsigned j = 0; j < height; ++j)
		{
			Row& row = rows[j];

			for (unsigned i = 0; i < width; ++i)
			{
				sf[i + j * width] = kriging.ordinaryKrigeSingleCell(sfKrigeParams, i, j, context);

				if (!nProgress.oneStep())
				{
					//process cancelled by user
					return false;
				}
			}
		}
	}

	if (hasColors)
	{
		for (unsigned char c = 0; c < 3; ++c)
		{
			// update the kriging value
			{
				size_t index = 0;
				for (unsigned j = 0; j < height; ++j)
				{
					const Row& row = rows[j];
					for (unsigned i = 0; i < width; ++i)
					{
						const ccRasterCell& cell = row[i];
						if (cell.nbPoints)
						{
							dataPoints[index++].value = cell.color.u[c];
						}
					}
				}
				assert(index == nonEmptyCellCount);
			}

			// compute default parameters
			Kriging::KrigeParams colorKrigeParams = kriging.computeDefaultParameters();
			// use the same model as for the altitudes
			if (krigeParams.model != Kriging::Invalid)
			{
				colorKrigeParams.model = krigeParams.model;
			}

			for (unsigned j = 0; j < height; ++j)
			{
				Row& row = rows[j];

				for (unsigned i = 0; i < width; ++i)
				{
					double col = kriging.ordinaryKrigeSingleCell(colorKrigeParams, i, j, context);
					ccRasterCell& cell = row[i];
					cell.color.u[c] = std::max(0.0, std::min(255.0, col));

					if (!nProgress.oneStep())
					{
						//process cancelled by user
						return false;
					}
				}
			}
		}
	}

	kriging.releaseOrdinaryKrigeContext(context);

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
	size_t emptyCellCount = 0;

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
			else
			{
				++emptyCellCount;
			}
		}
	}

	if (validCellCount)
	{
		meanHeight /= validCellCount;
	}
}


void ccRasterGrid::fillEmptyCells(	EmptyCellFillOption fillEmptyCellsStrategy,
									double customCellHeight/*=0.0*/)
{
	//fill empty cells
	if (fillEmptyCellsStrategy == LEAVE_EMPTY)
	{
		return;
	}

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
	case INTERPOLATE_DELAUNAY: // we still fill the empty cells, as some cells may not be interpolated in the end!
		defaultHeight = customCellHeight;
		break;
	case FILL_AVERAGE_HEIGHT:
		defaultHeight = meanHeight;
		break;
	case KRIGING:
		// nothing to do
		return;
	default:
		assert(false);
		return;
	}

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

	// update grid stats
	updateCellStats();
}

ccPointCloud* ccRasterGrid::convertToCloud(	bool exportHeightStats,
											bool exportSFStats,
											const std::vector<ExportableFields>& exportedStatistics,
											bool projectSFs,
											bool projectColors,
											bool resampleInputCloudXY,
											bool resampleInputCloudZ,
											ccGenericPointCloud* inputCloud,
											unsigned char Z,
											const ccBBox& box,
											double percentileValue,
											bool exportToOriginalCS,
											bool appendGridSizeToSFNames,
											ccProgressDialog* progressDialog/*=nullptr*/ ) const
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

	unsigned pointCount = validCellCount;
	if (pointCount == 0)
	{
		ccLog::Warning("[Rasterize] Empty grid!");
		return nullptr;
	}

	ccPointCloud* cloudGrid = nullptr;

	ccPointCloud* inputCloudAsPC = (inputCloud && inputCloud->isA(CC_TYPES::POINT_CLOUD)) ? static_cast<ccPointCloud*>(inputCloud) : nullptr;
	
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
					refCloud.addPointIndex(cell.nearestPointIndex);
				}
			}
		}

		assert(refCloud.size() != 0);
		cloudGrid = inputCloudAsPC ? inputCloudAsPC->partialClone(&refCloud) : ccPointCloud::From(&refCloud, inputCloud);
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

		if (!projectSFs)
		{
			//DGM: we can't stop right away: even if we have already resampled the
			//original cloud, we may have to create additional points and/or scalar fields
			//return cloudGrid;
		}
	}
	else
	{
		cloudGrid = new ccPointCloud("grid");
	}
	assert(cloudGrid);

	//shall we generate additional height data or scalar fields statistics?
	size_t numberOfExportedHeightStatisticsFields = 0;
	if (exportHeightStats)
	{
		numberOfExportedHeightStatisticsFields = exportedStatistics.size();
	}
	size_t maxNumberOfExportedSfStatisticsFields = 0;
	size_t numberOfExportedSfStatisticsFields = 0;
	size_t sfStatCount = exportedStatistics.size();
	if (exportSFStats && inputCloudAsPC && sfStatCount != 0)
	{
		maxNumberOfExportedSfStatisticsFields = inputCloudAsPC->getNumberOfScalarFields() * exportedStatistics.size();

		if (std::find(exportedStatistics.begin(), exportedStatistics.end(), PER_CELL_VALUE) != exportedStatistics.end())
		{
			// we ignore 'per cell value' which is not a statistic
			--sfStatCount;
		}
        numberOfExportedSfStatisticsFields = inputCloudAsPC->getNumberOfScalarFields() * sfStatCount;
    }
    
    std::vector<CCCoreLib::ScalarField*> exportedSFs;
	size_t totalNumberOfExportedFields = numberOfExportedHeightStatisticsFields + numberOfExportedSfStatisticsFields;
	if (totalNumberOfExportedFields != 0)
    {
		exportedSFs.reserve(totalNumberOfExportedFields);
    }
	
	//create some new SFs if needed (for the various statistics to compute)
	for (size_t k = 0; k < numberOfExportedHeightStatisticsFields + maxNumberOfExportedSfStatisticsFields; ++k)
	{
		size_t statIndex = 0;
		QString sfName;
		if (k < numberOfExportedHeightStatisticsFields)
		{
			statIndex = k;

			static const char* XYZ = "XYZ";
			sfName = XYZ[Z];

			if (exportedStatistics[k] == PER_CELL_VALUE)
				sfName += " values"; // use a simpler name
			else
				sfName += QChar(' ') + GetDefaultFieldName(exportedStatistics[statIndex]);
		}
		else
		{
			assert(sfStatCount != 0);
			assert(exportedStatistics.size() != 0);

			size_t indexOfSFStatsField = k - numberOfExportedHeightStatisticsFields;
			size_t sfIndex = indexOfSFStatsField / exportedStatistics.size();

			statIndex = indexOfSFStatsField - sfIndex * exportedStatistics.size();

			if (exportedStatistics[statIndex] == PER_CELL_VALUE)
			{
				// ignored
				continue;
			}

			sfName = QString::fromStdString(inputCloudAsPC->getScalarFieldName(static_cast<int>(sfIndex))) + " " + GetDefaultFieldName(exportedStatistics[statIndex]);
		}

		assert(statIndex < exportedStatistics.size());
		if (exportedStatistics[statIndex] == PER_CELL_PERCENTILE_VALUE)
		{
			sfName += QString(" P%1").arg(percentileValue);
		}

		if (appendGridSizeToSFNames)
		{
			sfName.append(QString(" (%1)").arg(gridStep));
		}

		// get or create the corresponding scalar field
		int sfIndex = cloudGrid->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIndex < 0)
		{
			sfIndex = cloudGrid->addScalarField(sfName.toStdString());
			if (sfIndex < 0)
			{
				ccLog::Warning("[Rasterize] Couldn't allocate scalar field(s)! Try to free some memory ...");
				delete cloudGrid;
				return nullptr;
			}
		}
		else
		{
			ccLog::Warning(QString("[Rasterize] Scalar field '%1' already exists. It will be overwritten.").arg(sfName));
		}

		exportedSFs.push_back(cloudGrid->getScalarField(sfIndex));
		assert(exportedSFs.back() != nullptr);
	}

	if (resampleInputCloudXY)
	{
		//if the cloud already has colors and we filled some empty cells, we must also add (black) colors
		projectColors = (cloudGrid->hasColors() && (validCellCount > nonEmptyCellCount));
	}
	else
	{
		//we need colors to project them!
		projectColors &= hasColors;
	}

	//the resampled cloud already contains the points corresponding to 'filled' cells so we will only
	//need to add the empty ones (if requested)
	if (!resampleInputCloudXY || (validCellCount > nonEmptyCellCount))
	{
		if (!cloudGrid->reserve(pointCount))
		{
			ccLog::Warning("[Rasterize] Not enough memory!");
			delete cloudGrid;
			return nullptr;
		}

		if (projectColors && !cloudGrid->reserveTheRGBTable())
		{
			ccLog::Warning("[Rasterize] Not enough memory to project colors!");
			cloudGrid->unallocateColors();
			projectColors = false;
		}
	}

	//horizontal dimensions
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	const unsigned char outX = (exportToOriginalCS ? X : 0);
	const unsigned char outY = (exportToOriginalCS ? Y : 1);
	const unsigned char outZ = (exportToOriginalCS ? Z : 2);

	//as the 'non empty cells points' are already in the cloud
	//we must take care of where we put the scalar fields values!
	unsigned nonEmptyCellIndex = 0;

	//we work with doubles as the grid step can be much smaller than the cloud coordinates!
	double Py = box.minCorner().u[Y]; //minCorner is the lower left cell CENTER

	if (progressDialog)
	{
		progressDialog->setMethodTitle(QObject::tr("Cloud export"));
		progressDialog->setInfo(QObject::tr("Exporting %1 fields").arg(totalNumberOfExportedFields));
		progressDialog->start();
		progressDialog->setAutoClose(false);
		progressDialog->show();
		QCoreApplication::processEvents();
	}

	// export points
	{
		QScopedPointer<CCCoreLib::NormalizedProgress> nProgress;
		if (progressDialog)
		{
			nProgress.reset(new CCCoreLib::NormalizedProgress(progressDialog, static_cast<unsigned>(height * width)));
		}

		std::vector<double> cellPointVal;
		std::vector<unsigned> cellPointIndexes;

		bool exportedStatisticsNeedSorting = false;
		for (size_t l = 0; l < exportedStatistics.size(); ++l)
		{
			switch (exportedStatistics[l])
			{
			case PER_CELL_MIN_VALUE:
			case PER_CELL_MAX_VALUE:
			case PER_CELL_VALUE_RANGE:
			case PER_CELL_MEDIAN_VALUE:
			case PER_CELL_PERCENTILE_VALUE:
			case PER_CELL_UNIQUE_COUNT_VALUE:
				exportedStatisticsNeedSorting = true;
				l = exportedStatistics.size(); // early stop
				break;
			default:
				// do nothing
				break;
			}
		}
		
		for (unsigned j = 0; j < height; ++j)
		{
			const ccRasterCell* aCell = rows[j].data();
			double Px = box.minCorner().u[X]; //minCorner is the lower left cell CENTER

			for (unsigned i = 0; i < width; ++i, ++aCell)
			{
				if (std::isfinite(aCell->h)) //valid cell (could have been filled or interpolated)
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

						if (projectColors)
						{
							ccColor::Rgb col(	static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.x)),
												static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.y)),
												static_cast<ColorCompType>(std::min(static_cast<double>(ccColor::MAX), aCell->color.z)));

							cloudGrid->addColor(col);
						}
					}

					//fill the associated SFs
					assert(!resampleInputCloudXY || inputCloud);
					assert(!resampleInputCloudXY || nonEmptyCellIndex < inputCloud->size()); //we can't be here if we have a fully resampled cloud! (resampleInputCloudXY implies that inputCloud is defined)
					assert(exportedSFs.size() >= numberOfExportedHeightStatisticsFields);

					bool cellPointIndexesBuilt = false;
					size_t sfIndex = 0;
					for (size_t k = 0; k < numberOfExportedHeightStatisticsFields + maxNumberOfExportedSfStatisticsFields; ++k)
					{
						CCCoreLib::ScalarField* sf = exportedSFs[sfIndex];
						ScalarType sVal = CCCoreLib::NAN_VALUE;

						// specific case: PER_CELL_VALUE
						if (k < numberOfExportedHeightStatisticsFields && exportedStatistics[k] == PER_CELL_VALUE)
						{
							sVal = aCell->h;
						}
						else
						{
							if (!cellPointIndexesBuilt) // only required the first time
							{
								aCell->getPointIndexes(cellPointIndexes, pointRefList);
								cellPointIndexesBuilt = true;
							}

							cellPointVal.clear();

							size_t statIndex = 0;
							if (k < numberOfExportedHeightStatisticsFields) // height statistics
							{
								statIndex = k;

								// Set up vector of height values for current cell 
								for (unsigned n = 0; n < aCell->nbPoints; ++n)
								{
									const CCVector3* P = inputCloud->getPoint(cellPointIndexes[n]);
									cellPointVal.push_back(P->u[Z]);
								}
							}
							else  // SF statistics
							{
								assert(exportedStatistics.size() != 0);
								size_t indexOfSFStatsField = k - numberOfExportedHeightStatisticsFields;
								size_t sfIndex = indexOfSFStatsField / exportedStatistics.size();

								statIndex = indexOfSFStatsField - sfIndex * exportedStatistics.size();

								if (exportedStatistics[statIndex] == PER_CELL_VALUE)
								{
									// ignored
									continue;
								}

								// Get input scalar field for statistics
								CCCoreLib::ScalarField* inputScalarField = inputCloudAsPC->getScalarField(static_cast<int>(sfIndex));

								// Set up vector of valid SF values for current cell 
								for (unsigned n = 0; n < aCell->nbPoints; ++n)
								{
									ScalarType sfValue = inputScalarField->getValue(cellPointIndexes[n]);
									if (std::isfinite(sfValue))
									{
										cellPointVal.push_back(sfValue);
									}
								}
							}

							if (exportedStatisticsNeedSorting)
							{
								//Sorting data in cell in ascending order
								ParallelSort(cellPointVal.begin(), cellPointVal.end(), [](ScalarType a, ScalarType b) { return a < b; });
							}

							if (cellPointVal.size())
							{
								switch (exportedStatistics[statIndex])
								{
								case PER_CELL_VALUE:
									assert(false);
									break;
								case PER_CELL_COUNT:
									sVal = static_cast<ScalarType>(cellPointVal.size());
									break;
								case PER_CELL_MIN_VALUE:
									sVal = static_cast<ScalarType>(cellPointVal.front());
									break;
								case PER_CELL_MAX_VALUE:
									sVal = static_cast<ScalarType>(cellPointVal.back());
									break;
								case PER_CELL_VALUE_RANGE:
									sVal = static_cast<ScalarType>(cellPointVal.back() - cellPointVal.front());
									break;
								case PER_CELL_AVG_VALUE:
									sVal = static_cast<ScalarType>(std::accumulate(cellPointVal.begin(), cellPointVal.end(), 0.0) / cellPointVal.size());
									break;
								case PER_CELL_MEDIAN_VALUE:
								{
									size_t midIndex = cellPointVal.size() / 2;
									if (cellPointVal.size() % 2) // odd number
									{
										sVal = static_cast<ScalarType>(cellPointVal[midIndex]);
									}
									else
									{
										sVal = static_cast<ScalarType>((cellPointVal[midIndex - 1] + cellPointVal[midIndex]) / 2);
									}
								}
								break;
								case PER_CELL_PERCENTILE_VALUE:
								{
									size_t index = static_cast<size_t>(percentileValue * cellPointVal.size() / 100.0);
									assert(index < cellPointVal.size());
									sVal = static_cast<ScalarType>(cellPointVal[std::min(index, cellPointVal.size() - 1)]);
								}
								break;
								case PER_CELL_UNIQUE_COUNT_VALUE:
								{
									size_t count = 0;
									for (size_t n = 1; n < cellPointVal.size(); ++n)
									{
										if (cellPointVal[n - 1] != cellPointVal[n])
										{
											++count;
										}
									}
									sVal = static_cast<ScalarType>(count);
								}
								break;
								case PER_CELL_VALUE_STD_DEV:
								{
									double cellSum = std::accumulate(cellPointVal.begin(), cellPointVal.end(), 0.0);
									double cellSquareSum = 0.0;
									for (size_t n = 0; n < cellPointVal.size(); n++)
									{
										cellSquareSum += cellPointVal[n] * cellPointVal[n];
									}
									double cellAvg = cellSum / cellPointVal.size();
									sVal = static_cast<ScalarType>(std::sqrt(std::max(0.0, cellSquareSum / cellPointVal.size() - cellAvg * cellAvg)));
								}
								break;
								default:
									assert(false);
									break;
								}
							}
						}
						
						if (resampleInputCloudXY)
						{
							if (aCell->nbPoints != 0)
							{
								//overwrite existing value of an already existing point
								assert(nonEmptyCellIndex < inputCloud->size());
								assert(nonEmptyCellIndex < sf->size());
								sf->setValue(nonEmptyCellIndex, sVal);
							}
							else
							{
								//new value for a new point (appended at the back of the SF)
								assert(sf->size() < sf->capacity());
								sf->addElement(sVal);
							}
						}
						else
						{
							//new value for a new point (appended at the back of the SF)
							assert(sf->size() < sf->capacity());
							sf->addElement(sVal);
						}

						++sfIndex;
					}

					if (aCell->nbPoints != 0)
					{
						++nonEmptyCellIndex;
					}
				}

				Px += gridStep;

				if (nProgress && !nProgress->oneStep())
				{
					//process cancelled by the user
					ccLog::Warning("[Rasterize] Cancelled by the user!");
					delete cloudGrid;
					return nullptr;
				}
			}

			Py += gridStep;
		}

		//finish the SFs initialization (if any)
		for (auto sf : exportedSFs)
		{
			if (sf)
			{
				assert(sf->size() == cloudGrid->size());
				sf->computeMinAndMax();
			}
		}
	}

	//take care of former scalar fields
	if (!resampleInputCloudXY)
	{
		//do we need to project the original SFs?
		if (projectSFs && inputCloud && inputCloudAsPC)
		{
			QScopedPointer<CCCoreLib::NormalizedProgress> nProgress;
			if (progressDialog)
			{
				progressDialog->setInfo(QObject::tr("Projecting %1 scalar fields").arg(scalarFields.size()));
				progressDialog->setValue(0);
				QCoreApplication::processEvents();
				nProgress.reset(new CCCoreLib::NormalizedProgress(progressDialog, static_cast<unsigned>(scalarFields.size())));
			}

			assert(scalarFields.size() == inputCloudAsPC->getNumberOfScalarFields());
			
			for (size_t k = 0; k < scalarFields.size(); ++k)
			{
				assert(!scalarFields[k].empty());

				//the corresponding SF should exist on the input cloud
				ccScalarField* formerSf = static_cast<ccScalarField*>(inputCloudAsPC->getScalarField(static_cast<int>(k)));
				assert(formerSf);
                
				//we try to create an equivalent SF on the output grid
				int sfIdx = cloudGrid->addScalarField(formerSf->getName());
				if (sfIdx < 0) //if we aren't lucky, the input cloud already had a SF with the same name
				{
					sfIdx = cloudGrid->addScalarField(formerSf->getName() + ".old");
				}

				if (sfIdx < 0)
				{
					ccLog::Warning("[Rasterize] Failed to allocate a new scalar field for storing SF '%s' values! Try to free some memory ...", formerSf->getName().c_str());
				}
				else
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloudGrid->getScalarField(sfIdx));
					assert(sf);
					//set sf values
					unsigned n = 0;
					const double* _sfGrid = scalarFields[k].data();
					for (unsigned j = 0; j < height; ++j)
					{
						const ccRasterGrid::Row& row = rows[j];
						for (unsigned i = 0; i < width; ++i, ++_sfGrid)
						{
							if (std::isfinite(row[i].h)) //valid cell (could have been filled or interpolated)
							{
								ScalarType s = static_cast<ScalarType>(*_sfGrid);
								sf->setValue(n++, s);
							}
						}
					}
					sf->computeMinAndMax();
					sf->importParametersFrom(formerSf);
					assert(sf->currentSize() == pointCount);
				}

				if (nProgress && !nProgress->oneStep())
				{
					//process cancelled by the user
					ccLog::Warning("[Rasterize] Cancelled by the user!");
					delete cloudGrid;
					return nullptr;
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
	cloudGrid->showSF(cloudGrid->hasScalarFields());

	return cloudGrid;
}
