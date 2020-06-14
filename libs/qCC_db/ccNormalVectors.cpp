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

#include "ccNormalVectors.h"

//Local
#include "ccSingleton.h"
#include "ccNormalCompressor.h"

//CCLib
#include <CCGeom.h>
#include <DgmOctreeReferenceCloud.h>
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>
#include <Neighbourhood.h>

//System
#include <assert.h>
#include <random>

//unique instance
static ccSingleton<ccNormalVectors> s_uniqueInstance;

//Number of points for local modeling to compute normals with 2D1/2 Delaunay triangulation
static const unsigned NUMBER_OF_POINTS_FOR_NORM_WITH_TRI = 6;
//Number of points for local modeling to compute normals with least square plane
static const unsigned NUMBER_OF_POINTS_FOR_NORM_WITH_LS = 3;
//Number of points for local modeling to compute normals with quadratic 'height' function
static const unsigned NUMBER_OF_POINTS_FOR_NORM_WITH_QUADRIC = 6;

ccNormalVectors* ccNormalVectors::GetUniqueInstance()
{
	if (!s_uniqueInstance.instance)
		s_uniqueInstance.instance = new ccNormalVectors();
	return s_uniqueInstance.instance;
}

void ccNormalVectors::ReleaseUniqueInstance()
{
	s_uniqueInstance.release();
}

ccNormalVectors::ccNormalVectors()
{
	init();
}

CompressedNormType ccNormalVectors::GetNormIndex(const PointCoordinateType N[])
{
	unsigned index = ccNormalCompressor::Compress(N);

	return static_cast<CompressedNormType>(index);
}

bool ccNormalVectors::enableNormalHSVColorsArray()
{
	if (!m_theNormalHSVColors.empty())
	{
		return true;
	}

	if (m_theNormalVectors.empty())
	{
		//'init' should be called first!
		return false;
	}

	try
	{
		m_theNormalHSVColors.resize(m_theNormalVectors.size());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	for (size_t i = 0; i < m_theNormalVectors.size(); ++i)
	{
		m_theNormalHSVColors[i] = ccNormalVectors::ConvertNormalToRGB(m_theNormalVectors[i]);
	}

	return true;
}

const ccColor::Rgb& ccNormalVectors::getNormalHSVColor(unsigned index) const
{
	assert(index < m_theNormalVectors.size());
	return m_theNormalHSVColors[index];
}

bool ccNormalVectors::init()
{
	unsigned numberOfVectors = ccNormalCompressor::NULL_NORM_CODE + 1;
	try
	{
		m_theNormalVectors.resize(numberOfVectors);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccNormalVectors::init] Not enough memory!");
		return false;
	}

	for (unsigned i = 0; i < numberOfVectors; ++i)
	{
		ccNormalCompressor::Decompress(i, m_theNormalVectors[i].u);
		m_theNormalVectors[i].normalize();
	}

	return true;
}

bool ccNormalVectors::UpdateNormalOrientations(	ccGenericPointCloud* theCloud,
												NormsIndexesTableType& theNormsCodes,
												Orientation preferredOrientation)
{
	assert(theCloud);

	//preferred orientation
	CCVector3 orientation(0,0,0);
	CCVector3 barycenter(0,0,0);
	bool useBarycenter = false;
	bool positiveSign = true;

	switch (preferredOrientation)
	{
	case PLUS_X:
	case MINUS_X:
	case PLUS_Y:
	case MINUS_Y:
	case PLUS_Z:
	case MINUS_Z:
		{
			//0-5 = +/-X,Y,Z
			assert(preferredOrientation >= 0 && preferredOrientation <= 5);

			orientation.u[preferredOrientation >> 1] = ((preferredOrientation & 1) == 0 ? PC_ONE : -PC_ONE); //odd number --> inverse direction
		}
		break;

	case PLUS_BARYCENTER:
	case MINUS_BARYCENTER:
		{
			barycenter = CCLib::GeometricalAnalysisTools::ComputeGravityCenter(theCloud);
			ccLog::Print(QString("[UpdateNormalOrientations] Barycenter: (%1,%2,%3)").arg(barycenter.x).arg(barycenter.y).arg(barycenter.z));
			useBarycenter = true;
			positiveSign = (preferredOrientation == 6);
		}
		break;

	case PLUS_ZERO:
	case MINUS_ZERO:
		{
			//barycenter = CCVector3(0,0,0);
			useBarycenter = true;
			positiveSign = (preferredOrientation == 8);
		}
		break;

	case PREVIOUS:
		{
			if (!theCloud->hasNormals())
			{
				ccLog::Warning("[UpdateNormalOrientations] Can't orient the new normals with the previous ones... as the cloud has no normals!");
				return false;
			}
		}
		break;

	default:
		assert(false);
		return false;
	}

	//we check each normal orientation
	for (unsigned i = 0; i < theNormsCodes.currentSize(); i++)
	{
		const CompressedNormType& code = theNormsCodes.getValue(i);
		CCVector3 N = GetNormal(code);

		if (preferredOrientation == PREVIOUS)
		{
			orientation = theCloud->getPointNormal(i);
		}
		else if (useBarycenter)
		{
			if (positiveSign)
			{
				orientation = *(theCloud->getPoint(i)) - barycenter;
			}
			else
			{
				orientation = barycenter - *(theCloud->getPoint(i));
			}
		}

		//we eventually check the sign
		if (N.dot(orientation) < 0)
		{
			//inverse normal and re-compress it
			N *= -1;
			theNormsCodes.setValue(i, ccNormalVectors::GetNormIndex(N.u));
		}
	}

	return true;
}

PointCoordinateType ccNormalVectors::GuessNaiveRadius(ccGenericPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return 0;
	}

	PointCoordinateType largetDim = cloud->getOwnBB().getMaxBoxDim();

	return largetDim / std::min<unsigned>(100, std::max<unsigned>(1, cloud->size()/100 ) );
}

PointCoordinateType ccNormalVectors::GuessBestRadius(	ccGenericPointCloud* cloud,
														CCLib::DgmOctree* inputOctree/*=0*/,
														CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (!cloud)
	{
		assert(false);
		return 0;
	}

	CCLib::DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new CCLib::DgmOctree(cloud);
		if (octree->build() <= 0)
		{
			delete octree;
			ccLog::Warning("[GuessBestRadius] Failed to compute the cloud octree");
			return 0;
		}
	}

	PointCoordinateType bestRadius = GuessNaiveRadius(cloud);
	if (bestRadius == 0)
	{
		ccLog::Warning("[GuessBestRadius] The cloud has invalid dimensions");
		return 0;
	}

	if (cloud->size() < 100)
	{
		//no need to do anything else for very small clouds!
		return bestRadius;
	}

	//we are now going to sample the cloud so as to compute statistics on the density
	{
		static const int s_aimedPop = 16;
		static const int s_aimedPopRange = 4;
		static const int s_minPop = 6;
		static const double s_minAboveMinRatio = 0.97;

		const unsigned sampleCount = std::min<unsigned>(200, cloud->size() / 10);

		double aimedPop = s_aimedPop;
		PointCoordinateType radius = bestRadius;
		PointCoordinateType lastRadius = radius;
		double lastMeanPop = 0;

		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);

		//we may have to do this several times
		for (size_t attempt = 0; attempt < 10; ++attempt)
		{
			int totalCount = 0;
			int totalSquareCount = 0;
			int minPop = 0;
			int maxPop = 0;
			int aboveMinPopCount = 0;

			unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);

			for (size_t i = 0; i < sampleCount; ++i)
			{
				unsigned randomIndex = dist(gen);
				assert(randomIndex < cloud->size());

				const CCVector3* P = cloud->getPoint(randomIndex);
				CCLib::DgmOctree::NeighboursSet Yk;
				int n = octree->getPointsInSphericalNeighbourhood(*P, radius, Yk, octreeLevel);
				assert(n >= 1);

				totalCount += n;
				totalSquareCount += n*n;
				if (i == 0)
				{
					minPop = maxPop = n;
				}
				else
				{
					if (n < minPop)
						minPop = n;
					else if (n > maxPop)
						maxPop = n;
				}

				if (n >= s_minPop)
				{
					++aboveMinPopCount;
				}
			}

			double meanPop = static_cast<double>(totalCount) / sampleCount;
			double stdDevPop = sqrt(fabs(static_cast<double>(totalSquareCount) / sampleCount - meanPop*meanPop));
			double aboveMinPopRatio = static_cast<double>(aboveMinPopCount) / sampleCount;

			ccLog::Print(QString("[GuessBestRadius] Radius = %1 -> samples population in [%2 ; %3] (mean %4 / std. dev. %5 / %6% above mininmum)")
										.arg(radius)
										.arg(minPop)
										.arg(maxPop)
										.arg(meanPop)
										.arg(stdDevPop)
										.arg(aboveMinPopRatio * 100)
						);

			if (fabs(meanPop - aimedPop) < s_aimedPopRange)
			{
				//we have found a correct radius
				bestRadius = radius;

				if (aboveMinPopRatio < s_minAboveMinRatio)
				{
					//ccLog::Warning("[GuessBestRadius] The cloud density is very inhomogeneous! You may have to increase the radius to get valid normals everywhere... but the result will be smoother");
					aimedPop = s_aimedPop + (2.0*stdDevPop)/* * (1.0-aboveMinPopRatio)*/;
					assert(aimedPop >= s_aimedPop);
				}
				else
				{
					break;
				}
			}

			//otherwise we have to find a better estimate for the radius
			PointCoordinateType newRadius = radius;
			//(warning: we consider below that the number of points is proportional to the SURFACE of the neighborhood)
			assert(meanPop >= 1.0);
			if (attempt == 0)
			{
				//this is our best (only) guess for the moment
				bestRadius = radius;

				newRadius = radius * sqrt(aimedPop / meanPop);
			}
			else
			{
				//keep track of our best guess nevertheless
				if (fabs(meanPop - aimedPop) < fabs(bestRadius - aimedPop))
				{
					bestRadius = radius;
				}

				double slope = (radius*radius - lastRadius*lastRadius) / (meanPop - lastMeanPop);
				PointCoordinateType newSquareRadius = lastRadius*lastRadius + (aimedPop - lastMeanPop) * slope;
				if (newSquareRadius > 0)
				{
					newRadius = sqrt(newSquareRadius);
				}
				else
				{
					//can't do any better!
					break;
				}
			}

			lastRadius = radius;
			lastMeanPop = meanPop;

			radius = newRadius;
		}
	}

	if (octree && !inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	return bestRadius;
}

bool ccNormalVectors::ComputeCloudNormals(	ccGenericPointCloud* theCloud,
											NormsIndexesTableType& theNormsCodes,
											CC_LOCAL_MODEL_TYPES localModel,
											PointCoordinateType localRadius,
											Orientation preferredOrientation/*=UNDEFINED*/,
											CCLib::GenericProgressCallback* progressCb/*=0*/,
											CCLib::DgmOctree* inputOctree/*=0*/)
{
	assert(theCloud);

	unsigned pointCount = theCloud->size();
	if (pointCount < 3)
	{
		return false;
	}

	CCLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(theCloud);
		if (theOctree->build() <= 0)
		{
			delete theOctree;
			return false;
		}
	}

	//reserve some memory to store the (compressed) normals
	if (!theNormsCodes.isAllocated() || theNormsCodes.currentSize() < pointCount)
	{
		if (!theNormsCodes.resizeSafe(pointCount))
		{
			if (theOctree && !inputOctree)
				delete theOctree;
			return false;
		}
	}

	//we instantiate 3D normal vectors
	NormsTableType* theNorms = new NormsTableType;
	static const CCVector3 blankN(0, 0, 0);
	if (!theNorms->resizeSafe(pointCount, true, &blankN))
	{
		theNormsCodes.resize(0);
		if (theOctree && !inputOctree)
			delete theOctree;
		return false;
	}
	//theNorms->fill(0);

	void* additionalParameters[2] = { reinterpret_cast<void*>(theNorms), reinterpret_cast<void*>(&localRadius) };

	unsigned processedCells = 0;
	switch (localModel)
	{
	case LS:
		{
			unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(localRadius);
			processedCells = theOctree->executeFunctionForAllCellsAtLevel(	level,
																			&(ComputeNormsAtLevelWithLS),
																			additionalParameters,
																			true,
																			progressCb,
																			"Normals Computation[LS]");
		}
		break;
	case TRI:
		{
			unsigned char level = theOctree->findBestLevelForAGivenPopulationPerCell(NUMBER_OF_POINTS_FOR_NORM_WITH_TRI);
			processedCells = theOctree->executeFunctionForAllCellsStartingAtLevel(	level,
																					&(ComputeNormsAtLevelWithTri),
																					additionalParameters,
																					NUMBER_OF_POINTS_FOR_NORM_WITH_TRI / 2,
																					NUMBER_OF_POINTS_FOR_NORM_WITH_TRI * 3,
																					true,
																					progressCb,
																					"Normals Computation[TRI]");
		}
		break;
	case QUADRIC:
		{
			unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(localRadius);
			processedCells = theOctree->executeFunctionForAllCellsAtLevel(	level,
																			&(ComputeNormsAtLevelWithQuadric),
																			additionalParameters,
																			true,
																			progressCb,
																			"Normals Computation[QUADRIC]");
		}
		break;

	default:
		break;
	}

	//error or canceled by user?
	if (processedCells == 0 || (progressCb && progressCb->isCancelRequested()))
	{
		theNormsCodes.resize(0);
		return false;
	}

	//we 'compress' each normal
	std::fill(theNormsCodes.begin(), theNormsCodes.end(), 0);
	for (unsigned i = 0; i < pointCount; i++)
	{
		const CCVector3& N = theNorms->at(i);
		CompressedNormType nCode = GetNormIndex(N);
		theNormsCodes.setValue(i, nCode);
	}

	theNorms->release();
	theNorms = nullptr;

	//preferred orientation
	if (preferredOrientation != UNDEFINED)
	{
		UpdateNormalOrientations(theCloud, theNormsCodes, preferredOrientation);
	}

	if (theOctree && !inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return true;
}

bool ccNormalVectors::ComputeNormalWithQuadric(CCLib::GenericIndexedCloudPersist* points, const CCVector3& P, CCVector3& N)
{
	CCLib::Neighbourhood Z(points);

	Tuple3ub dims;
	const PointCoordinateType* h = Z.getQuadric(&dims);
	if (h)
	{
		const CCVector3* gv = Z.getGravityCenter();
		assert(gv);

		const unsigned char& iX = dims.x;
		const unsigned char& iY = dims.y;
		const unsigned char& iZ = dims.z;

		PointCoordinateType lX = P.u[iX] - gv->u[iX];
		PointCoordinateType lY = P.u[iY] - gv->u[iY];

		N.u[iX] = h[1] + (2 * h[3] * lX) + (h[4] * lY);
		N.u[iY] = h[2] + (2 * h[5] * lY) + (h[4] * lX);
		N.u[iZ] = -1;

		//normalize the result
		N.normalize();

		return true;
	}
	else
	{
		return false;
	}
}

bool ccNormalVectors::ComputeNormalWithLS(CCLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N)
{
	N = CCVector3(0, 0, 0);

	if (!pointAndNeighbors)
	{
		assert(false);
		return false;
	}

	if (pointAndNeighbors->size() < 3)
	{
		return false;
	}

	CCLib::Neighbourhood Z(pointAndNeighbors);
	const CCVector3* _N = Z.getLSPlaneNormal();
	if (_N)
	{
		N = *_N;
		return true;
	}
	else
	{
		return false;
	}
}


bool ccNormalVectors::ComputeNormalWithTri(CCLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N)
{
	N = CCVector3(0, 0, 0);

	if (!pointAndNeighbors)
	{
		assert(false);
		return false;
	}

	if (pointAndNeighbors->size() < 3)
	{
		return false;
	}

	CCLib::Neighbourhood Z(pointAndNeighbors);

	//we mesh the neighbour points (2D1/2)
	CCLib::GenericIndexedMesh* theMesh = Z.triangulateOnPlane();
	if (!theMesh)
	{
		return false;
	}

	unsigned triCount = theMesh->size();

	//for all triangles
	theMesh->placeIteratorAtBeginning();
	for (unsigned j = 0; j < triCount; ++j)
	{
		//we can't use getNextTriangleVertIndexes (which is faster on mesh groups but not multi-thread compatible) but anyway we'll never get mesh groups here!
		const CCLib::VerticesIndexes* tsi = theMesh->getTriangleVertIndexes(j);

		//we look if the central point is one of the triangle's vertices
		if (tsi->i1 == 0 || tsi->i2 == 0 || tsi->i3 == 0)
		{
			const CCVector3 *A = pointAndNeighbors->getPoint(tsi->i1);
			const CCVector3 *B = pointAndNeighbors->getPoint(tsi->i2);
			const CCVector3 *C = pointAndNeighbors->getPoint(tsi->i3);

			CCVector3 no = (*B - *A).cross(*C - *A);
			//no.normalize();
			N += no;
		}
	}

	delete theMesh;
	theMesh = nullptr;

	//normalize the 'mean' vector
	N.normalize();

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithQuadric(	const CCLib::DgmOctree::octreeCell& cell,
														void** additionalParameters,
														CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius, cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	for (unsigned j = 0; j < pointCount; ++j, ++it)
	{
		it->point = cell.points->getPointPersistentPtr(j);
		it->pointIndex = cell.points->getPointGlobalIndex(j);
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i = 0; i < pointCount; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, false);
		float cur_radius = radius;
		while (k < NUMBER_OF_POINTS_FOR_NORM_WITH_QUADRIC && cur_radius < 16*radius)
		{
			cur_radius *= 1.189207115f;
			k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, cur_radius, false);
		}
		if (k >= NUMBER_OF_POINTS_FOR_NORM_WITH_QUADRIC)
		{
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

			CCVector3 N;
			if (ComputeNormalWithQuadric(&neighbours, nNSS.queryPoint, N))
			{
				theNorms->setValue(cell.points->getPointGlobalIndex(i), N);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithLS(const CCLib::DgmOctree::octreeCell& cell,
												void** additionalParameters,
												CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius, cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	{
		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j = 0; j < pointCount; ++j, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i = 0; i < pointCount; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, radius, false);
		float cur_radius = radius;
		while (k < NUMBER_OF_POINTS_FOR_NORM_WITH_LS && cur_radius < 16*radius)
		{
			cur_radius *= 1.189207115f;
			k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, cur_radius, false);
		}
		if (k >= NUMBER_OF_POINTS_FOR_NORM_WITH_LS)
		{
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

			CCVector3 N;
			if (ComputeNormalWithLS(&neighbours, N))
			{
				theNorms->setValue(cell.points->getPointGlobalIndex(i), N);
			}
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithTri(	const CCLib::DgmOctree::octreeCell& cell,
													void** additionalParameters,
													CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);

	CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.minNumberOfNeighbors = NUMBER_OF_POINTS_FOR_NORM_WITH_TRI;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	{
		for (unsigned j = 0; j < pointCount; ++j, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i = 0; i < pointCount; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		unsigned k = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS);
		if (k > NUMBER_OF_POINTS_FOR_NORM_WITH_TRI)
		{
			if (k > NUMBER_OF_POINTS_FOR_NORM_WITH_TRI * 3)
				k = NUMBER_OF_POINTS_FOR_NORM_WITH_TRI * 3;
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

			CCVector3 N;
			if (ComputeNormalWithTri(&neighbours, N))
			{
				theNorms->setValue(cell.points->getPointGlobalIndex(i), N);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

QString ccNormalVectors::ConvertStrikeAndDipToString(double& strike_deg, double& dip_deg)
{
	int iStrike = static_cast<int>(strike_deg);
	int iDip = static_cast<int>(dip_deg);

	return QString("N%1°E - %2°").arg(iStrike, 3, 10, QChar('0')).arg(iDip, 3, 10, QChar('0'));
}

QString ccNormalVectors::ConvertDipAndDipDirToString(PointCoordinateType dip_deg, PointCoordinateType dipDir_deg)
{
	int iDipDir = static_cast<int>(dipDir_deg);
	int iDip = static_cast<int>(dip_deg);

	return QString("Dip: %1 deg. - Dip direction: %2 deg.").arg(iDip, 3, 10, QChar('0')).arg(iDipDir, 3, 10, QChar('0'));
}

void ccNormalVectors::ConvertNormalToStrikeAndDip(const CCVector3& N, PointCoordinateType& strike_deg, PointCoordinateType& dip_deg)
{
	// Adapted from Andy Michael's 'stridip.c':
	// Finds strike and dip of plane given normal vector having components n, e, and u
	// output is in degrees north of east and then
	// uses a right hand rule for the dip of the plane
	if (N.norm2() > std::numeric_limits<PointCoordinateType>::epsilon())
	{
		strike_deg = 180.0 - atan2(N.y, N.x)*CC_RAD_TO_DEG;		//atan2 output is between -180 and 180! So strike is always positive here
		PointCoordinateType x = sqrt(N.x*N.x + N.y*N.y);		//x is the horizontal magnitude
		dip_deg = atan2(x, N.z)*CC_RAD_TO_DEG;
	}
	else
	{
		strike_deg = dip_deg = std::numeric_limits<PointCoordinateType>::quiet_NaN();
	}
}

void ccNormalVectors::ConvertNormalToDipAndDipDir(const CCVector3& N, PointCoordinateType& dip_deg, PointCoordinateType& dipDir_deg)
{
	//http://en.wikipedia.org/wiki/Structural_geology#Geometries

	if (N.norm2d() > std::numeric_limits<PointCoordinateType>::epsilon())
	{
		// The dip direction must be the same for parallel facets, regardless
		// of whether their normals point upwards or downwards.
		//
		// The formula using atan2() with the swapped N.x and N.y already
		// gives the correct results for facets with the normal pointing
		// upwards, so just use the sign of N.z to invert the normals if they
		// point downwards.
		double Nsign = N.z < 0 ? -1.0 : 1.0; //DGM: copysign is not available on VS2012

		//"Dip direction is measured in 360 degrees, generally clockwise from North"
		double dipDir_rad = atan2(Nsign * N.x, Nsign * N.y); //result in [-pi,+pi]
		if (dipDir_rad < 0)
		{
			dipDir_rad += 2 * M_PI;
		}

		// Dip angle
		//
		// acos() returns values in [0, pi] but using fabs() all the normals
		// are considered pointing upwards, so the actual result will be in
		// [0, pi/2] as required by the definition of dip.
		// We skip the division by r because the normal is a unit vector.
		double dip_rad = acos(fabs(N.z));

		dipDir_deg = static_cast<PointCoordinateType>(dipDir_rad * CC_RAD_TO_DEG);
		dip_deg = static_cast<PointCoordinateType>(dip_rad * CC_RAD_TO_DEG);
	}
	else
	{
		dipDir_deg = dip_deg = std::numeric_limits<PointCoordinateType>::quiet_NaN();
	}
}

CCVector3 ccNormalVectors::ConvertDipAndDipDirToNormal(PointCoordinateType dip_deg, PointCoordinateType dipDir_deg, bool upward/*=true*/)
{
	//specific case
	if (std::isnan(dip_deg) || std::isnan(dipDir_deg))
	{
		return CCVector3(0, 0, 0);
	}
	
	double Nz = cos(dip_deg * CC_DEG_TO_RAD);
	double Nxy = sqrt(1.0 - Nz * Nz);
	double dipDir_rad = dipDir_deg * CC_DEG_TO_RAD;
	CCVector3 N(	static_cast<PointCoordinateType>(Nxy * sin(dipDir_rad)),
					static_cast<PointCoordinateType>(Nxy * cos(dipDir_rad)),
					static_cast<PointCoordinateType>(Nz) );

#ifdef _DEBUG
	//internal consistency test
	PointCoordinateType dip2, dipDir2;
	ConvertNormalToDipAndDipDir(N, dip2, dipDir2);
	assert(fabs(dip2 - dip_deg) < 1.0e-3 && (dip2 == 0 || fabs(dipDir2 - dipDir_deg) < 1.0e-3));
#endif

	if (!upward)
	{
		N = -N;
	}
	return N;
}

void ccNormalVectors::ConvertNormalToHSV(const CCVector3& N, float& H, float& S, float& V)
{
	PointCoordinateType dip = 0;
	PointCoordinateType dipDir = 0;
	ConvertNormalToDipAndDipDir(N, dip, dipDir);

	H = static_cast<float>(dipDir);
	if (H == 360.0f) //H is in [0;360[
		H = 0;
	S = static_cast<float>(dip / 90); //S is in [0;1]
	V = 1.0f;
}

ccColor::Rgb ccNormalVectors::ConvertNormalToRGB(const CCVector3& N)
{
	ccColor::Rgbf col((N.x + 1) / 2, (N.y + 1) / 2, (N.z + 1) / 2);
	return ccColor::Rgb(	static_cast<ColorCompType>(col.r * ccColor::MAX),
							static_cast<ColorCompType>(col.g * ccColor::MAX),
							static_cast<ColorCompType>(col.b * ccColor::MAX));
}
