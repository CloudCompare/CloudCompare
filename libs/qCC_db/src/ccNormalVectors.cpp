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
#include "ccHObjectCaster.h"
#include "ccSensor.h"

//CCCoreLib
#include <CCGeom.h>
#include <DgmOctreeReferenceCloud.h>
#include <GenericIndexedMesh.h>
#include <GenericProgressCallback.h>
#include <GeometricalAnalysisTools.h>
#include <Neighbourhood.h>

//System
#include <cassert>

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
	CCVector3 prefOrientation(0, 0, 0);
	CCVector3 originPoint(0, 0, 0);
	bool useOriginPoint = false;
	bool fromOriginPoint = true;

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

			prefOrientation.u[preferredOrientation >> 1] = ((preferredOrientation & 1) == 0 ? CCCoreLib::PC_ONE : -CCCoreLib::PC_ONE); //odd number --> inverse direction
		}
		break;

	case PLUS_BARYCENTER:
	case MINUS_BARYCENTER:
		{
			originPoint = CCCoreLib::GeometricalAnalysisTools::ComputeGravityCenter(theCloud);
			ccLog::Print(QString("[UpdateNormalOrientations] Barycenter: (%1;%2;%3)").arg(originPoint.x).arg(originPoint.y).arg(originPoint.z));
			useOriginPoint = true;
			fromOriginPoint = (preferredOrientation == PLUS_BARYCENTER);
		}
		break;

	case PLUS_ORIGIN:
	case MINUS_ORIGIN:
		{
			originPoint = CCVector3(0, 0, 0);
			useOriginPoint = true;
			fromOriginPoint = (preferredOrientation == PLUS_ORIGIN);
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

	case MINUS_SENSOR_ORIGIN:
	case PLUS_SENSOR_ORIGIN:
		{
			// look for the first sensor (child) with a valid origin
			bool sensorFound = false;
			for (unsigned i = 0; i < theCloud->getChildrenNumber(); ++i)
			{
				ccHObject* child = theCloud->getChild(i);
				if (child && child->isKindOf(CC_TYPES::SENSOR))
				{
					ccSensor* sensor = ccHObjectCaster::ToSensor(child);
					if (sensor->getActiveAbsoluteCenter(originPoint))
					{
						useOriginPoint = true;
						fromOriginPoint = (preferredOrientation == PLUS_SENSOR_ORIGIN);
						sensorFound = true;
						break;
					}
				}
			}
			if (!sensorFound)
			{
				ccLog::Warning("[UpdateNormalOrientations] Could not find a valid sensor child");
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
			prefOrientation = theCloud->getPointNormal(i);
		}
		else if (useOriginPoint)
		{
			if (fromOriginPoint)
			{
				prefOrientation = *(theCloud->getPoint(i)) - originPoint;
			}
			else
			{
				prefOrientation = originPoint - *(theCloud->getPoint(i));
			}
		}

		//we eventually check the sign
		if (N.dot(prefOrientation) < 0)
		{
			//inverse normal and re-compress it
			N *= -1;
			theNormsCodes.setValue(i, ccNormalVectors::GetNormIndex(N.u));
		}
	}

	return true;
}

bool ccNormalVectors::ComputeCloudNormals(	ccGenericPointCloud* theCloud,
											NormsIndexesTableType& theNormsCodes,
											CCCoreLib::LOCAL_MODEL_TYPES localModel,
											PointCoordinateType localRadius,
											Orientation preferredOrientation/*=UNDEFINED*/,
											CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/,
											CCCoreLib::DgmOctree* inputOctree/*=nullptr*/)
{
	assert(theCloud);

	unsigned pointCount = theCloud->size();
	if (pointCount < 3)
	{
		return false;
	}

	CCCoreLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCCoreLib::DgmOctree(theCloud);
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
			if (nullptr == inputOctree)
			{
				delete theOctree;
			}
			return false;
		}
	}

	//we instantiate 3D normal vectors
	NormsTableType* theNorms = new NormsTableType;
	static const CCVector3 blankN(0, 0, 0);
	if (!theNorms->resizeSafe(pointCount, true, &blankN))
	{
		theNormsCodes.resize(0);
		if (nullptr == inputOctree)
		{
			delete theOctree;
		}
		return false;
	}
	//theNorms->fill(0);

	void* additionalParameters[2] = { reinterpret_cast<void*>(theNorms), reinterpret_cast<void*>(&localRadius) };

	unsigned processedCells = 0;
	switch (localModel)
	{
	case CCCoreLib::LS:
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
	case CCCoreLib::TRI:
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
	case CCCoreLib::QUADRIC:
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

	if (nullptr == inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return true;
}

bool ccNormalVectors::ComputeNormalWithQuadric(CCCoreLib::GenericIndexedCloudPersist* points, const CCVector3& P, CCVector3& N)
{
	CCCoreLib::Neighbourhood Z(points);

	CCCoreLib::SquareMatrix toLocalOrientation;
	const PointCoordinateType* h = Z.getQuadric(&toLocalOrientation);
	if (h)
	{
		const CCVector3* gv = Z.getGravityCenter();
		assert(gv);
		
		CCVector3 Q = toLocalOrientation * (P - *gv);

		N.x = h[1] + (2 * h[3] * Q.x) + (h[4] * Q.y);
		N.y = h[2] + (2 * h[5] * Q.y) + (h[4] * Q.x);
		N.z = -1;

		N = toLocalOrientation.inv() * N;

		//normalize the result
		N.normalize();

		return true;
	}
	else
	{
		return false;
	}
}

bool ccNormalVectors::ComputeNormalWithLS(CCCoreLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N)
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

	CCCoreLib::Neighbourhood Z(pointAndNeighbors);
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


bool ccNormalVectors::ComputeNormalWithTri(CCCoreLib::GenericIndexedCloudPersist* pointAndNeighbors, CCVector3& N)
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

	CCCoreLib::Neighbourhood Z(pointAndNeighbors);

	//we mesh the neighbour points (2D1/2)
	std::string errorStr;
	
	CCCoreLib::GenericIndexedMesh* theMesh = Z.triangulateOnPlane( CCCoreLib::Neighbourhood::DO_NOT_DUPLICATE_VERTICES,
																   CCCoreLib::Neighbourhood::IGNORE_MAX_EDGE_LENGTH,
																   errorStr );
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
		const CCCoreLib::VerticesIndexes* tsi = theMesh->getTriangleVertIndexes(j);

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

bool ccNormalVectors::ComputeNormsAtLevelWithQuadric(	const CCCoreLib::DgmOctree::octreeCell& cell,
														void** additionalParameters,
														CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	CCCoreLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
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
			CCCoreLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

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

bool ccNormalVectors::ComputeNormsAtLevelWithLS(const CCCoreLib::DgmOctree::octreeCell& cell,
												void** additionalParameters,
												CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	{
		CCCoreLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
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
			CCCoreLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

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

bool ccNormalVectors::ComputeNormsAtLevelWithTri(	const CCCoreLib::DgmOctree::octreeCell& cell,
													void** additionalParameters,
													CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);

	CCCoreLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.minNumberOfNeighbors = NUMBER_OF_POINTS_FOR_NORM_WITH_TRI;
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned pointCount = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(pointCount);
	CCCoreLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
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
			CCCoreLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood, k);

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
	int iDip = static_cast<int>(std::round(dip_deg));

	return QString("N%1°E - %2°").arg(iStrike, 3, 10, QChar('0')).arg(iDip, 3, 10, QChar('0'));
}

QString ccNormalVectors::ConvertDipAndDipDirToString(PointCoordinateType dip_deg, PointCoordinateType dipDir_deg)
{
	int iDipDir = static_cast<int>(std::round(dipDir_deg));
	int iDip = static_cast<int>(std::round(dip_deg));

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
		strike_deg = 180.0 - CCCoreLib::RadiansToDegrees( atan2(N.y, N.x) );		//atan2 output is between -180 and 180! So strike is always positive here
		PointCoordinateType x = sqrt(N.x*N.x + N.y*N.y);		//x is the horizontal magnitude
		dip_deg = CCCoreLib::RadiansToDegrees( atan2(x, N.z) );
	}
	else
	{
		strike_deg = dip_deg = std::numeric_limits<PointCoordinateType>::quiet_NaN();
	}
}

void ccNormalVectors::ConvertNormalToDipAndDipDir(const CCVector3f& N, float& dip_deg, float& dipDir_deg)
{
	//http://en.wikipedia.org/wiki/Structural_geology#Geometries

	if (N.norm2() > std::numeric_limits<float>::epsilon())
	{
		// The dip direction must be the same for parallel facets, regardless
		// of whether their normals point upwards or downwards.
		//
		// The formula using atan2() with the swapped N.x and N.y already
		// gives the correct results for facets with the normal pointing
		// upwards, so just use the sign of N.z to invert the normals if they
		// point downwards.
		float Nsign = N.z < 0 ? -1.0f : 1.0f; //DGM: copysign is not available on VS2012

		//"Dip direction is measured in 360 degrees, generally clockwise from North"
		float dipDir_rad = atan2(Nsign * N.x, Nsign * N.y); //result in [-pi,+pi]
		if (dipDir_rad < 0)
		{
			dipDir_rad += static_cast<float>(2 * M_PI);
		}

		// Dip angle
		//
		// acos() returns values in [0, pi] but using std::abs() all the normals
		// are considered pointing upwards, so the actual result will be in
		// [0, pi/2] as required by the definition of dip.
		// We skip the division by r because the normal is a unit vector.
		float dip_rad = acos(std::abs(N.z));

		dipDir_deg = CCCoreLib::RadiansToDegrees(dipDir_rad);
		dip_deg = CCCoreLib::RadiansToDegrees(dip_rad);
	}
	else
	{
		dipDir_deg = dip_deg = std::numeric_limits<float>::quiet_NaN();
	}
}

void ccNormalVectors::ConvertNormalToDipAndDipDir(const CCVector3d& N, double& dip_deg, double& dipDir_deg)
{
	//http://en.wikipedia.org/wiki/Structural_geology#Geometries

	if (N.norm2d() > std::numeric_limits<double>::epsilon())
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
		// acos() returns values in [0, pi] but using std::abs() all the normals
		// are considered pointing upwards, so the actual result will be in
		// [0, pi/2] as required by the definition of dip.
		// We skip the division by r because the normal is a unit vector.
		double dip_rad = acos(std::abs(N.z));

		dipDir_deg = CCCoreLib::RadiansToDegrees(dipDir_rad);
		dip_deg = CCCoreLib::RadiansToDegrees(dip_rad);
	}
	else
	{
		dipDir_deg = dip_deg = std::numeric_limits<double>::quiet_NaN();
	}
}

CCVector3f ccNormalVectors::ConvertDipAndDipDirToNormal(float dip_deg, float dipDir_deg, bool upward/*=true*/)
{
	//specific case
	if (std::isnan(dip_deg) || std::isnan(dipDir_deg))
	{
		return CCVector3f(0, 0, 0);
	}
	
	float Nz = cos(CCCoreLib::DegreesToRadians(dip_deg));
	float Nxy = sqrt(1.0f - Nz * Nz);
	float dipDir_rad = CCCoreLib::DegreesToRadians(dipDir_deg);
	CCVector3f N(	Nxy * sin(dipDir_rad),
					Nxy * cos(dipDir_rad),
					Nz );

#ifdef _DEBUG
	//internal consistency test
	float dip2, dipDir2;
	ConvertNormalToDipAndDipDir(N, dip2, dipDir2);
	assert(std::abs(dip2 - dip_deg) < 1.0e-3f && (dip2 == 0 || std::abs(dipDir2 - dipDir_deg) < 1.0e-3f));
#endif

	if (!upward)
	{
		N = -N;
	}
	return N;
}

CCVector3d ccNormalVectors::ConvertDipAndDipDirToNormal(double dip_deg, double dipDir_deg, bool upward/*=true*/)
{
	//specific case
	if (std::isnan(dip_deg) || std::isnan(dipDir_deg))
	{
		return CCVector3(0, 0, 0);
	}

	double Nz = cos(CCCoreLib::DegreesToRadians(dip_deg));
	double Nxy = sqrt(1.0 - Nz * Nz);
	double dipDir_rad = CCCoreLib::DegreesToRadians(dipDir_deg);
	CCVector3d N(	Nxy * sin(dipDir_rad),
					Nxy * cos(dipDir_rad),
					Nz );

#ifdef _DEBUG
	//internal consistency test
	double dip2, dipDir2;
	ConvertNormalToDipAndDipDir(N, dip2, dipDir2);
	assert(std::abs(dip2 - dip_deg) < 1.0e-3 && (dip2 == 0 || std::abs(dipDir2 - dipDir_deg) < 1.0e-3));
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
