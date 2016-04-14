//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "GeometricalAnalysisTools.h"

//local
#include "GenericIndexedCloudPersist.h"
#include "Neighbourhood.h"
#include "ReferenceCloud.h"
#include "GenericProgressCallback.h"
#include "GenericCloud.h"
#include "DistanceComputationTools.h"
#include "DgmOctreeReferenceCloud.h"
#include "ScalarField.h"
#include "ScalarFieldTools.h"

//system
#include <assert.h>
#include <random>

using namespace CCLib;

int GeometricalAnalysisTools::computeCurvature(	GenericIndexedCloudPersist* theCloud,
												Neighbourhood::CC_CURVATURE_TYPE cType,
												PointCoordinateType kernelRadius,
												GenericProgressCallback* progressCb/*=0*/,
												DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
		return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints < 5)
		return -2;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[2] = {	static_cast<void*>(&cType),
										static_cast<void*>(&kernelRadius) };

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(level,
													&computeCellCurvatureAtLevel,
													additionalParameters,
													true,
													progressCb,
													"Curvature Computation") == 0)
	{
		//something went wrong
		result = -4;
	}

	if (!inputOctree)
        delete theOctree;

	return result;
}

//"PER-CELL" METHOD: CURVATURE ESTIMATION (WITH QUADRATIC FIT)
//ADDITIONAL PARAMETERS (2):
// [0] -> (CC_CURVATURE_TYPE*) cType : curvature type
// [1] -> (PointCoordinateType*) radius : sphere radius
bool GeometricalAnalysisTools::computeCellCurvatureAtLevel(	const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress/*=0*/)
{
	//parameters
	Neighbourhood::CC_CURVATURE_TYPE cType	= *static_cast<Neighbourhood::CC_CURVATURE_TYPE*>(additionalParameters[0]);
	PointCoordinateType radius				= *static_cast<PointCoordinateType*>(additionalParameters[1]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//we already know some of the neighbours: the points in the current cell!
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}

		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i=0; i<n; ++i,++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	//for each point in the cell
	for (unsigned i=0; i<n; ++i)
	{
		ScalarType curv = NAN_VALUE;

		cell.points->getPoint(i,nNSS.queryPoint);

		//look for neighbors in a sphere
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
		unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		//neighborCount = std::min(neighborCount,16);

		if (neighborCount > 5)
		{
			//current point index
			unsigned index = cell.points->getPointGlobalIndex(i);
			//current point index in neighbourhood (to compute curvature at the right position!)
			unsigned indexInNeighbourhood = 0;

			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood,neighborCount);
			Neighbourhood Z(&neighboursCloud);

			//look for local index
			for (unsigned j=0;j<neighborCount;++j)
			{
				if (nNSS.pointsInNeighbourhood[j].pointIndex == index)
				{
					indexInNeighbourhood = j;
					break;
				}
			}

			curv = Z.computeCurvature(indexInNeighbourhood,cType);
		}

		cell.points->setPointScalarValue(i,curv);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

int GeometricalAnalysisTools::flagDuplicatePoints(	GenericIndexedCloudPersist* theCloud,
													double minDistanceBetweenPoints/*=1.0e-12*/,
													GenericProgressCallback* progressCb/*=0*/,
													DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
		return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints <= 1)
		return -2;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();
	//set all flags to 0 by default
	theCloud->forEach(CCLib::ScalarFieldTools::SetScalarValueToZero);

	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(minDistanceBetweenPoints));

	//parameters
	void* additionalParameters[1] = { static_cast<void*>(&minDistanceBetweenPoints) };

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&flagDuplicatePointsInACellAtLevel,
														additionalParameters,
														false, //doesn't work in parallel!
														progressCb,
														"Flag duplicate points") == 0)
	{
		//something went wrong
		result = -4;
	}

	if (!inputOctree)
		delete theOctree;

	return result;
}

//"PER-CELL" METHOD: FLAG DUPLICATE POINTS
//ADDITIONAL PARAMETERS (1):
// [0] -> (double*) maxSquareDistBetweenPoints: max square distance between points
bool GeometricalAnalysisTools::flagDuplicatePointsInACellAtLevel(	const DgmOctree::octreeCell& cell,
																	void** additionalParameters,
																	NormalizedProgress* nProgress/*=0*/)
{
	//parameter(s)
	double minDistBetweenPoints = *static_cast<double*>(additionalParameters[0]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(static_cast<PointCoordinateType>(minDistBetweenPoints),cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell
	
	//for each point in the cell
	for (unsigned i=0; i<n; ++i)
	{
		//don't process points already flagged as 'duplicate'
		if (cell.points->getPointScalarValue(i) == 0)
		{
			cell.points->getPoint(i,nNSS.queryPoint);

			//look for neighbors in a sphere
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
			unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,minDistBetweenPoints,false);
			if (neighborCount > 1) //the point itself lies in the neighborhood
			{
				unsigned iIndex = cell.points->getPointGlobalIndex(i);
				for (unsigned j=0; j<neighborCount; ++j)
				{
					if (nNSS.pointsInNeighbourhood[j].pointIndex != iIndex)
					{
						//flag this point as 'duplicate'
						cell.points->getAssociatedCloud()->setPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex,static_cast<ScalarType>(1));
					}
				}
			}
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

int GeometricalAnalysisTools::computeLocalDensityApprox(GenericIndexedCloudPersist* theCloud,
														Density densityType,
														GenericProgressCallback* progressCb/*=0*/,
														DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
		return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints < 3)
		return -2;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	//determine best octree level to perform the computation
	unsigned char level = theOctree->findBestLevelForAGivenPopulationPerCell(3);

	//parameters
	void* additionalParameters[] = { static_cast<void*>(&densityType) };

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&computeApproxPointsDensityInACellAtLevel,
														additionalParameters,
														true,
														progressCb,
														"Approximate Local Density Computation") == 0)
	{
		//something went wrong
		result = -4;
	}

	if (!inputOctree)
		delete theOctree;

	return result;
}

//volume of a unit sphere
static double s_UnitSphereVolume = 4.0 * M_PI / 3.0;

//"PER-CELL" METHOD: APPROXIMATE LOCAL DENSITY
//ADDITIONAL PARAMETERS (0): NONE
bool GeometricalAnalysisTools::computeApproxPointsDensityInACellAtLevel(const DgmOctree::octreeCell& cell,
																		void** additionalParameters,
																		NormalizedProgress* nProgress/*=0*/)
{
	//extract additional parameter(s)
	Density densityType = *static_cast<Density*>(additionalParameters[0]);
	
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.minNumberOfNeighbors				= 2;
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size();
	for (unsigned i=0; i<n; ++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		//the first point is always the point itself!
		if (cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS) > 1)
		{
			double R2 = nNSS.pointsInNeighbourhood[1].squareDistd;

			ScalarType density = NAN_VALUE;
			if (R2 > ZERO_TOLERANCE)
			{
				switch (densityType)
				{
				case DENSITY_KNN:
					{
						//we return in fact the (inverse) distance to the nearest neighbor
						density = static_cast<ScalarType>(1.0 / sqrt(R2));
					}
					break;
				case DENSITY_2D:
					{
						//circle area (2D approximation)
						double circleArea = M_PI * R2;
						density = static_cast<ScalarType>(1.0 / circleArea);
					}
					break;
				case DENSITY_3D:
					{
						//sphere area
						double sphereArea =  s_UnitSphereVolume * R2 * sqrt(R2);
						density = static_cast<ScalarType>(1.0 / sphereArea);
					}
					break;
				default:
					assert(false);
					break;
				}
			}
			cell.points->setPointScalarValue(i,density);
		}
		else
		{
			//shouldn't happen! Apart if the cloud has only one point...
			cell.points->setPointScalarValue(i,NAN_VALUE);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

int GeometricalAnalysisTools::computeLocalDensity(	GenericIndexedCloudPersist* theCloud,
													Density densityType,
													PointCoordinateType kernelRadius,
													GenericProgressCallback* progressCb/*=0*/,
													DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
		return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints < 3)
		return -2;

	//compute the right dimensional coef based on the expected output
	double dimensionalCoef = 1.0;
	switch (densityType)
	{
	case DENSITY_KNN:
		dimensionalCoef = 1.0;
		break;
	case DENSITY_2D:
		dimensionalCoef = M_PI * (static_cast<double>(kernelRadius) * kernelRadius);
		break;
	case DENSITY_3D:
		dimensionalCoef = s_UnitSphereVolume * ((static_cast<double>(kernelRadius) * kernelRadius) * kernelRadius);
		break;
	default:
		assert(false);
		return -5;
	}

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	//determine best octree level to perform the computation
	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[] = {	static_cast<void*>(&kernelRadius),
										static_cast<void*>(&dimensionalCoef) };

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&computePointsDensityInACellAtLevel,
														additionalParameters,
														true,
														progressCb,
														"Local Density Computation") == 0)
	{
		//something went wrong
		result = -4;
	}

	if (!inputOctree)
        delete theOctree;

	return result;
}

//"PER-CELL" METHOD: LOCAL DENSITY
//ADDITIONNAL PARAMETERS (2):
// [0] -> (PointCoordinateType*) kernelRadius : spherical neighborhood radius
// [1] -> (ScalarType*) sphereVolume : spherical neighborhood volume
bool GeometricalAnalysisTools::computePointsDensityInACellAtLevel(	const DgmOctree::octreeCell& cell, 
																	void** additionalParameters,
																	NormalizedProgress* nProgress/*=0*/)
{
	//parameter(s)
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[0]);
	double dimensionalCoef = *static_cast<double*>(additionalParameters[1]);
	
	assert(dimensionalCoef > 0);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell
	
	//for each point in the cell
	for (unsigned i=0; i<n; ++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		//look for neighbors inside a sphere
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
		unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

		ScalarType density = static_cast<ScalarType>(neighborCount/dimensionalCoef);
		cell.points->setPointScalarValue(i,density);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

int GeometricalAnalysisTools::computeRoughness(GenericIndexedCloudPersist* theCloud, PointCoordinateType kernelRadius, GenericProgressCallback* progressCb/*=0*/, DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
		return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints < 3)
		return -2;

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[1] = { static_cast<void*>(&kernelRadius) };

	int result = 0;

	if (theOctree->executeFunctionForAllCellsAtLevel(	level,
														&computePointsRoughnessInACellAtLevel,
														additionalParameters,
														true,
														progressCb,
														"Roughness Computation") == 0)
	{
		//something went wrong
		result = -4;
	}

	if (!inputOctree)
		delete theOctree;

	return result;
}

//"PER-CELL" METHOD: ROUGHNESS ESTIMATION (LEAST SQUARES PLANE FIT)
//ADDITIONNAL PARAMETERS (1):
// [0] -> (PointCoordinateType*) kernelRadius : neighbourhood radius
bool GeometricalAnalysisTools::computePointsRoughnessInACellAtLevel(const DgmOctree::octreeCell& cell, 
																	void** additionalParameters,
																	NormalizedProgress* nProgress/*=0*/)
{
	//parameter(s)
	PointCoordinateType radius = *static_cast<PointCoordinateType*>(additionalParameters[0]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell
	
	//for each point in the cell
	for (unsigned i=0; i<n; ++i)
	{
		ScalarType d = NAN_VALUE;
		cell.points->getPoint(i,nNSS.queryPoint);

		//look for neighbors inside a sphere
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (= neighborCount)!
		unsigned neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		if (neighborCount > 3)
		{
			//find the query point in the nearest neighbors set and place it at the end
			const unsigned globalIndex = cell.points->getPointGlobalIndex(i);
			unsigned localIndex = 0;
			while (localIndex < neighborCount && nNSS.pointsInNeighbourhood[localIndex].pointIndex != globalIndex)
				++localIndex;
			//the query point should be in the nearest neighbors set!
			assert(localIndex < neighborCount);
			if (localIndex+1 < neighborCount) //no need to swap with another point if it's already at the end!
			{
				std::swap(nNSS.pointsInNeighbourhood[localIndex],nNSS.pointsInNeighbourhood[neighborCount-1]);
			}

			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood,neighborCount-1); //we don't take the query point into account!
			Neighbourhood Z(&neighboursCloud);

			const PointCoordinateType* lsPlane = Z.getLSPlane();
			if (lsPlane)
				d = fabs(DistanceComputationTools::computePoint2PlaneDistance(&nNSS.queryPoint,lsPlane));

			//swap the points back to their original position (DGM: not necessary)
			//if (localIndex+1 < neighborCount)
			//{
			//	std::swap(nNSS.pointsInNeighbourhood[localIndex],nNSS.pointsInNeighbourhood[neighborCount-1]);
			//}
		}

		cell.points->setPointScalarValue(i,d);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

CCVector3 GeometricalAnalysisTools::computeGravityCenter(GenericCloud* theCloud)
{
	assert(theCloud);
	
	unsigned count = theCloud->size();
	if (count == 0)
		return CCVector3();

	CCVector3d sum(0,0,0);

	theCloud->placeIteratorAtBegining();
	const CCVector3 *P = 0;
	while ((P = theCloud->getNextPoint()))
	{
		sum += CCVector3d::fromArray(P->u);
	}

	sum /= static_cast<double>(count);
	return CCVector3::fromArray(sum.u);
}

CCVector3 GeometricalAnalysisTools::computeWeightedGravityCenter(GenericCloud* theCloud, ScalarField* weights)
{
	assert(theCloud && weights);

	unsigned count = theCloud->size();
	if (count == 0 || !weights || weights->currentSize() < count)
		return CCVector3();

	CCVector3d sum(0, 0, 0);

	theCloud->placeIteratorAtBegining();
	double wSum = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = theCloud->getNextPoint();
		ScalarType w = weights->getValue(i);
		if (!ScalarField::ValidValue(w))
			continue;
		sum += CCVector3d::fromArray(P->u) * fabs(w);
		wSum += w;
	}

	if (wSum != 0)
		sum /= wSum;

	return CCVector3::fromArray(sum.u);
}

CCLib::SquareMatrixd GeometricalAnalysisTools::computeCovarianceMatrix(GenericCloud* theCloud, const PointCoordinateType* _gravityCenter)
{
	assert(theCloud);
	unsigned n = (theCloud ? theCloud->size() : 0);
	if (n==0)
		return CCLib::SquareMatrixd();

	CCLib::SquareMatrixd covMat(3);
	covMat.clear();

	//gravity center
	CCVector3 G = (_gravityCenter ?  CCVector3(_gravityCenter) : computeGravityCenter(theCloud));

	//cross sums (we use doubles to avoid overflow)
	double mXX = 0;
	double mYY = 0;
	double mZZ = 0;
	double mXY = 0;
	double mXZ = 0;
	double mYZ = 0;

	theCloud->placeIteratorAtBegining();
	for (unsigned i=0;i<n;++i)
	{
		const CCVector3* Q = theCloud->getNextPoint();

		CCVector3 P = *Q-G;
		mXX += static_cast<double>(P.x*P.x);
		mYY += static_cast<double>(P.y*P.y);
		mZZ += static_cast<double>(P.z*P.z);
		mXY += static_cast<double>(P.x*P.y);
		mXZ += static_cast<double>(P.x*P.z);
		mYZ += static_cast<double>(P.y*P.z);
	}

	covMat.m_values[0][0] = mXX/static_cast<double>(n);
	covMat.m_values[0][0] = mYY/static_cast<double>(n);
	covMat.m_values[0][0] = mZZ/static_cast<double>(n);
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY/static_cast<double>(n);
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ/static_cast<double>(n);
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ/static_cast<double>(n);

	return covMat;
}

CCLib::SquareMatrixd GeometricalAnalysisTools::computeCrossCovarianceMatrix(GenericCloud* P,
																			GenericCloud* Q,
																			const CCVector3& Gp,
																			const CCVector3& Gq)
{
	assert(P && Q);
	assert(Q->size() == P->size());

	//shortcuts to output matrix lines
	CCLib::SquareMatrixd covMat(3);
	double* l1 = covMat.row(0);
	double* l2 = covMat.row(1);
	double* l3 = covMat.row(2);

	P->placeIteratorAtBegining();
	Q->placeIteratorAtBegining();

	//sums
	unsigned count = P->size();
	for (unsigned i=0; i<count; i++)
	{
		CCVector3 Pt = *P->getNextPoint() - Gp;
		CCVector3 Qt = *Q->getNextPoint() - Gq;

        l1[0] += Pt.x*Qt.x;
        l1[1] += Pt.x*Qt.y;
        l1[2] += Pt.x*Qt.z;
        l2[0] += Pt.y*Qt.x;
        l2[1] += Pt.y*Qt.y;
        l2[2] += Pt.y*Qt.z;
        l3[0] += Pt.z*Qt.x;
        l3[1] += Pt.z*Qt.y;
        l3[2] += Pt.z*Qt.z;
	}

	covMat.scale(1.0/static_cast<double>(count));

	return covMat;
}

CCLib::SquareMatrixd GeometricalAnalysisTools::computeWeightedCrossCovarianceMatrix(GenericCloud* P, //data
																					GenericCloud* Q, //model
																					const CCVector3& Gp,
																					const CCVector3& Gq,
																					ScalarField* coupleWeights/*=0*/)
{
    assert(P && Q);
	assert(Q->size() == P->size());
	assert(coupleWeights);
	assert(coupleWeights->currentSize() == P->size());

	//shortcuts to output matrix lines
	CCLib::SquareMatrixd covMat(3);
	double* r1 = covMat.row(0);
	double* r2 = covMat.row(1);
	double* r3 = covMat.row(2);

	P->placeIteratorAtBegining();
	Q->placeIteratorAtBegining();

	//sums
	unsigned count = P->size();
	double wSum = 0.0; //we will normalize by the sum
	for (unsigned i = 0; i<count; i++)
	{
		CCVector3d Pt = CCVector3d::fromArray((*P->getNextPoint() - Gp).u);
		CCVector3 Qt = *Q->getNextPoint() - Gq;

		//Weighting scheme for cross-covariance is inspired from
		//https://en.wikipedia.org/wiki/Weighted_arithmetic_mean#Weighted_sample_covariance
		double wi = 1.0;
		if (coupleWeights)
		{
			ScalarType w = coupleWeights->getValue(i);
			if (!ScalarField::ValidValue(w))
				continue;
			wi = fabs(w);
		}

		//DGM: we virtually make the P (data) point nearer if it has a lower weight
		Pt *= wi;
		wSum += wi;

		//1st row
		r1[0] += Pt.x * Qt.x;
		r1[1] += Pt.x * Qt.y;
		r1[2] += Pt.x * Qt.z;
		//2nd row
		r2[0] += Pt.y * Qt.x;
		r2[1] += Pt.y * Qt.y;
		r2[2] += Pt.y * Qt.z;
		//3rd row
		r3[0] += Pt.z * Qt.x;
		r3[1] += Pt.z * Qt.y;
		r3[2] += Pt.z * Qt.z;
	}

	if (wSum != 0.0)
		covMat.scale(1.0/wSum);

	return covMat;
}

bool GeometricalAnalysisTools::refineSphereLS(	GenericIndexedCloudPersist* cloud,
												CCVector3& center,
												PointCoordinateType& radius,
												double minReltaiveCenterShift/*=1.0e-3*/)
{
	if (!cloud || cloud->size() < 5)
	{
		//invalid input
		return false;
	}
	
	CCVector3d c = CCVector3d::fromArray(center.u);

	unsigned count = cloud->size();

	//compute barycenter
	CCVector3d G(0,0,0);
	{
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			G += CCVector3d::fromArray(P->u);
		}
		G /= count;
	}

	static const unsigned MAX_ITERATIONS = 100;
	for (unsigned it=0; it<MAX_ITERATIONS; ++it)
	{
		// Compute average L, dL/da, dL/db, dL/dc.
		double meanNorm = 0.0;
		CCVector3d derivatives(0,0,0);
		unsigned realCount = 0;
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* Pi = cloud->getPoint(i);
			CCVector3d Di = CCVector3d::fromArray(Pi->u) - c;
			double norm = Di.norm();
			if (norm < ZERO_TOLERANCE)
				continue;

			meanNorm += norm;
			derivatives = Di/norm;
			++realCount;
		}

		meanNorm /= count;
		derivatives /= count;

		//backup previous center
		CCVector3d c0 = c;
		//deduce new center
		c = G - derivatives * meanNorm;
		double r = meanNorm;

		double shift = (c-c0).norm();
		double relativeShift = shift/r;
		if (relativeShift < minReltaiveCenterShift)
			break;
	}

	return true;
}

bool GeometricalAnalysisTools::detectSphereRobust(	GenericIndexedCloudPersist* cloud,
													double outliersRatio,
													CCVector3& center,
													PointCoordinateType& radius,
													double& rms,
													GenericProgressCallback* progressCb/*=0*/,
													double confidence/*=0.99*/)
{
	if (!cloud || cloud->size() < 4)
	{
		//invalid input
		return false;
	}
	assert(confidence < 1.0);
	confidence = std::min(confidence,1.0-FLT_EPSILON);

	const unsigned p = 4;
	unsigned n = cloud->size();

	//we'll need an array (sorted) to compute the medians
	std::vector<PointCoordinateType> values;
	try
	{
		values.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//number of samples
	unsigned m = 1;
	if (n > p)
		m = static_cast<unsigned>( log(1.0-confidence) / log(1.0-pow(1.0-outliersRatio,static_cast<double>(p))) );

	//for progress notification
	NormalizedProgress nProgress(progressCb, m);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[64];
			sprintf(buffer, "Least Median of Squares samples: %u", m);
			progressCb->setInfo(buffer);
			progressCb->setMethodTitle("Detect sphere");
		}
		progressCb->update(0);
		progressCb->start();
	}

	//now we are going to randomly extract a subset of 4 points and test the resulting sphere each time
	std::random_device rd;   // non-deterministic generator
	std::mt19937 gen(rd());  // to seed mersenne twister.
	std::uniform_int_distribution<unsigned> dist(0, n - 1);
	unsigned sampleCount = 0;
	unsigned attempts = 0;
	double minError = -1.0;
	while (sampleCount < m && attempts < 2*m)
	{
		//get 4 random (different) indexes
		unsigned indexes[4] = {0,0,0,0};
		for (unsigned j=0; j<4; ++j)
		{
			bool isOK = false;
			while (!isOK)
			{
				indexes[j] = dist(gen);
				isOK = true;
				for (unsigned k=0; k<j && isOK; ++k)
					if (indexes[j] == indexes[k])
						isOK = false;
			}
		}

		const CCVector3* A = cloud->getPoint(indexes[0]);
		const CCVector3* B = cloud->getPoint(indexes[1]);
		const CCVector3* C = cloud->getPoint(indexes[2]);
		const CCVector3* D = cloud->getPoint(indexes[3]);
		
		++attempts;
		CCVector3 thisCenter;
		PointCoordinateType thisRadius;
		if (!computeSphereFrom4(*A,*B,*C,*D,thisCenter,thisRadius))
			continue;

		//compute residuals
		for (unsigned i=0; i<n; ++i)
		{
			PointCoordinateType error = (*cloud->getPoint(i) - thisCenter).norm() - thisRadius;
			values[i] = error*error;
		}
		std::sort(values.begin(), values.end());

		//the error is the median of the squared residuals
		double error = values[n/2];

		//we keep track of the solution with the least error
		if (error < minError || minError < 0.0)
		{
			minError = error;
			center = thisCenter;
			radius = thisRadius;
		}

		++sampleCount;

		if (progressCb && !nProgress.oneStep())
		{
			//progress canceled by the user
			return false;
		}
	}

	//too many failures?!
	if (sampleCount < m)
	{
		return false;
	}
	
	//last step: robust estimation
	ReferenceCloud candidates(cloud);
	if (n > p)
	{
		//e robust standard deviation estimate (see Zhang's report)
		double sigma = 1.4826 * (1.0 + 5.0 /(n-p)) * sqrt(minError);

		//compute the least-squares best-fitting sphere with the points
		//having residuals below 2.5 sigma
		double maxResidual = 2.5 * sigma;
		if (candidates.reserve(n))
		{
			//compute residuals and select the points
			for (unsigned i=0; i<n; ++i)
			{
				PointCoordinateType error = (*cloud->getPoint(i) - center).norm() - radius;
				if (error < maxResidual)
					candidates.addPointIndex(i);
			}
			candidates.resize(candidates.size());
			
			//eventually estimate the robust sphere parameters with least squares (iterative)
			if (refineSphereLS(&candidates,center,radius))
			{
				//replace input cloud by this subset!
				cloud = &candidates;
				n = cloud->size();
			}
		}
		else
		{
			//not enough memory!
			//we'll keep the rough estimate...
		}
	}

	//update residuals
	{
		double residuals = 0;
		for (unsigned i=0; i<n; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			double e = (*P - center).norm() - radius;
			residuals += e*e;
		}
		rms = sqrt(residuals/n);
	}
	
	return true;
}

//******************************************************************************
//
//  Purpose:
//
//    DMAT_SOLVE uses Gauss-Jordan elimination to solve an N by N linear system.
//
//  Discussion:
//
//    The doubly dimensioned array A is treated as a one dimensional vector,
//    stored by COLUMNS.  Entry A(I,J) is stored as A[I+J*N]
//
//  Modified:
//
//    29 August 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the order of the matrix.
//
//    Input, int RHS_NUM, the number of right hand sides.  RHS_NUM
//    must be at least 0.
//
//    Input/output, double A[N*(N+RHS_NUM)], contains in rows and columns 1
//    to N the coefficient matrix, and in columns N+1 through
//    N+RHS_NUM, the right hand sides.  On output, the coefficient matrix
//    area has been destroyed, while the right hand sides have
//    been overwritten with the corresponding solutions.
//
//    Output, int DMAT_SOLVE, singularity flag.
//    0, the matrix was not singular, the solutions were computed;
//    J, factorization failed on step J, and the solutions could not
//    be computed.
//
int dmat_solve ( int n, int rhs_num, double a[] )
{
	for (int j = 0; j < n; j++ )
	{
		//  Choose a pivot row.
		int ipivot = j;
		double apivot = a[j+j*n];

		for (int i = j; i < n; i++ )
		{
			if ( fabs(apivot) < fabs(a[i+j*n]) )
			{
				apivot = a[i+j*n];
				ipivot = i;
			}
		}

		if ( apivot == 0.0 )
		{
			return j;
		}

		//  Interchange.
		for (int i = 0; i < n + rhs_num; i++ )
		{
			std::swap(a[ipivot+i*n], a[j+i*n]);
		}

		//  A(J,J) becomes 1.
		a[j+j*n] = 1.0;
		for (int k = j; k < n + rhs_num; k++ )
		{
			a[j+k*n] = a[j+k*n] / apivot;
		}

		//  A(I,J) becomes 0.
		for (int i = 0; i < n; i++ )
		{
			if ( i != j )
			{
				double factor = a[i+j*n];
				a[i+j*n] = 0.0;
				for (int k = j; k < n + rhs_num; k++ )
				{
					a[i+k*n] = a[i+k*n] - factor * a[j+k*n];
				}
			}
		}
	}

	return 0;
}

bool GeometricalAnalysisTools::computeSphereFrom4(	const CCVector3& A,
													const CCVector3& B,
													const CCVector3& C,
													const CCVector3& D,
													CCVector3& center,
													PointCoordinateType& radius )
{
	//inspired from 'tetrahedron_circumsphere_3d' by Adrian Bowyer and John Woodwark

	//Set up the linear system.
	double a[12];
	{
		CCVector3 AB = B-A;
		a[0] = AB.x;
		a[3] = AB.y;
		a[6] = AB.z;
		a[9] = AB.norm2d();
	}
	{
		CCVector3 AC = C-A;
		a[1]  = AC.x;
		a[4]  = AC.y;
		a[7]  = AC.z;
		a[10] = AC.norm2d();
	}
	{
		CCVector3 AD = D-A;
		a[2]  = AD.x;
		a[5]  = AD.y;
		a[8]  = AD.z;
		a[11] = AD.norm2d();
	}

	//  Solve the linear system (with Gauss-Jordan elimination)
	if ( dmat_solve ( 3, 1, a ) != 0 )
	{
		//system is singular?
		return false;
	}

	//  Compute the radius and center.
	CCVector3 u = CCVector3(static_cast<PointCoordinateType>(a[0+3*3]),
							static_cast<PointCoordinateType>(a[1+3*3]),
							static_cast<PointCoordinateType>(a[2+3*3])) / 2;
	radius = u.norm();
	center = A + u;

	return true;
}