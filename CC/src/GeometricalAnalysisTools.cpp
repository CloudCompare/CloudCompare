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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "GeometricalAnalysisTools.h"

#include "GenericIndexedCloudPersist.h"
#include "Neighbourhood.h"
#include "ReferenceCloud.h"
#include "GenericProgressCallback.h"
#include "GenericCloud.h"
#include "DistanceComputationTools.h"
#include "DgmOctreeReferenceCloud.h"
#include "ScalarField.h"

#include <assert.h>

using namespace CCLib;

//#define COMPUTE_CURVATURE_2
int GeometricalAnalysisTools::computeCurvature(GenericIndexedCloudPersist* theCloud, Neighbourhood::CC_CURVATURE_TYPE cType, float kernelRadius, GenericProgressCallback* progressCb, DgmOctree* _theOctree)
{
	if (!theCloud)
        return -1;

	unsigned numberOfPoints = theCloud->size();
#ifdef COMPUTE_CURVATURE_2
	if (numberOfPoints<5)
#else
	if (numberOfPoints<10)
#endif
        return -2;

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	uchar level = theOctree->DgmOctree::findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[2];
	additionalParameters[0] = (void*)&cType;
	additionalParameters[1] = (void*)&kernelRadius;

	int result = 0;

#ifndef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtLevel(level,
#else
	if (theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#endif
													&computeCellCurvatureAtLevel,
													additionalParameters,
													progressCb,
													"Curvature Computation")==0)
	{
		//something went wrong
		result = -4;
	}

	if (!_theOctree)
        delete theOctree;

	return result;
}

//FONCTION "CELLULAIRE" DE CALCUL DE COURBURE (PAR FONCTION DE HAUTEUR LOCALE)
//DETAIL DES PARAMETRES ADDITIONNELS (2) :
// [0] -> (CC_CURVATURE_TYPE*) cType : curvature type
// [1] -> (float*) radius : sphere radius
bool GeometricalAnalysisTools::computeCellCurvatureAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//parameters
	Neighbourhood::CC_CURVATURE_TYPE cType	= *((Neighbourhood::CC_CURVATURE_TYPE*)additionalParameters[0]);
	float radius							= *((float*)additionalParameters[1]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.truncatedCellCode					= cell.truncatedCode;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);
	//*/

	unsigned i,j,k,n = cell.points->size(); //number of points in the current cell

	//we already know some of the neighbours: the points in the current cell!
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	for (i=0;i<n;++i,++it)
	{
		it->point = cell.points->getPointPersistentPtr(i);
		it->pointIndex = cell.points->getPointGlobalIndex(i);
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	unsigned index,indexInNeighbourhood;
	DistanceType curv;

	//for each point in the cell
	for (i=0;i<n;++i)
	{
#ifndef COMPUTE_CURVATURE_2
		curv = HIDDEN_VALUE;
#else
        curv = BIG_VALUE;
#endif

		cell.points->getPoint(i,nNSS.queryPoint);

		//look for neighbors in a sphere
		k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		//k = ccMin(k,16);

#ifndef COMPUTE_CURVATURE_2
		if (k>5)
#else
		if (k>10)
#endif
		{
		    //current point index
			index = cell.points->getPointGlobalIndex(i);
		    //current point index in neighbourhood (to compute curvature at thre right position!)
            indexInNeighbourhood = 0;

			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood,k);
			Neighbourhood Z(&neighboursCloud);

			//look for local index
			for (j=0;j<k;++j)
				if (nNSS.pointsInNeighbourhood[j].pointIndex == index)
				{
                    indexInNeighbourhood = j;
					break;
				}

#ifndef COMPUTE_CURVATURE_2
			curv = Z.computeCurvature(indexInNeighbourhood,cType);
#else
			curv = Z.computeCurvature2(indexInNeighbourhood,cType);
#endif
		}

		cell.points->setPointScalarValue(i,curv);
	}

	return true;
}

int GeometricalAnalysisTools::computeLocalDensity(GenericIndexedCloudPersist* theCloud, GenericProgressCallback* progressCb, DgmOctree* _theOctree)
{
	if (!theCloud)
        return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints<3)
        return -2;

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	uchar level = theOctree->findBestLevelForAGivenPopulationPerCell(3);

	int result = 0;

#ifndef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtLevel(level,
#else
	if (theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#endif
														&computePointsDensityInACellAtLevel,
														0,
														progressCb,
														"Local Density Computation")==0)
	{
		//something went wrong
		result = -4;
	}

	if (!_theOctree)
        delete theOctree;

	return result;
}

//FONCTION "CELLULAIRE" DE CALCUL DE DENSITE LOCALE
//PAS DE PARAMETRES ADDITIONNELS
const DistanceType c_sphereVolumeCoef=(DistanceType)(4.0/3.0*M_PI);
bool GeometricalAnalysisTools::computePointsDensityInACellAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.minNumberOfNeighbors				= 2;
	nNSS.truncatedCellCode					= cell.truncatedCode;
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned i,n=cell.points->size();
	for (i=0;i<n;++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

        //the first point is always the point itself!
		if (cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS)>1)
		{
            //DGM: we consider that the point is alone in a sphere of radius R and volume V=(4*pi/3)*R^3
			//So, the local density is ~1/V!
            DistanceType R2 = nNSS.pointsInNeighbourhood[1].squareDist; //R2 in fact
            DistanceType V = R2*sqrt(R2)*c_sphereVolumeCoef; //R^3 * (4*pi/3)
			cell.points->setPointScalarValue(i,(DistanceType)1.0/std::max(V,(DistanceType)ZERO_TOLERANCE));
		}
		else
		{
			//shoudln't happen! Appart if the cloud has only one point...
            cell.points->setPointScalarValue(i,HIDDEN_VALUE);
		}
	}

	return true;
}

int GeometricalAnalysisTools::computeRoughness(GenericIndexedCloudPersist* theCloud, float kernelRadius, GenericProgressCallback* progressCb/*=0*/, DgmOctree* _theOctree/*=0*/)
{
	if (!theCloud)
        return -1;

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints<3)
        return -2;

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -3;
		}
	}

	theCloud->enableScalarField();

	uchar level = theOctree->DgmOctree::findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);

	//parameters
	void* additionalParameters[1];
	additionalParameters[0] = (void*)&kernelRadius;

	int result = 0;

#ifdef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#else
	if (theOctree->executeFunctionForAllCellsAtLevel(level,
#endif
		&computePointsRoughnessInACellAtLevel,
		additionalParameters,
		progressCb,
		"Roughness Computation")==0)
	{
		//something went wrong
		result = -4;
	}

	if (!_theOctree)
        delete theOctree;

	return result;
}


//FONCTION "CELLULAIRE" DE CALCUL DE RUGOSITE (PAR PLAN AUX MOINDRES CARRES)
//DETAIL DES PARAMETRES ADDITIONNELS (1) :
// [0] -> (float*) kernelRadius : le rayon du voisinage de calcul
bool GeometricalAnalysisTools::computePointsRoughnessInACellAtLevel(const DgmOctree::octreeCell& cell, 
																	void** additionalParameters)
{
	//parameter(s)
	float radius = *((float*)additionalParameters[0]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.truncatedCellCode					= cell.truncatedCode;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	unsigned i,n = cell.points->size(); //number of points in the current cell
	
	//we already know some of the neighbours: the points in the current cell!
	/*try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (const std::bad_alloc&) //out of memory
	{
		return false;
	}

	DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();

	//whether cell is completely inside spherical neighbourhood or not
	//bool completelyInside = (nNSS.cellCenter - 

	for (i=0;i<n;++i,++it)
	{
		it->pointIndex = cell.points->getPointGlobalIndex(i);
		it->point = cell.points->getAssociatedCloud()->getPointPersistentPtr(it->pointIndex);
		//it->squareDist = 
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;
	//*/

	//for each point in the cell
	for (i=0;i<n;++i)
	{
        DistanceType d = HIDDEN_VALUE;
		cell.points->getPoint(i,nNSS.queryPoint);

		//look for neighbors in a sphere
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		if (k>2)
		{
			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood,k);
			Neighbourhood Z(&neighboursCloud);

            const PointCoordinateType* lsq = Z.getLSQPlane();
            if (lsq)
                d = DistanceComputationTools::computePoint2PlaneDistance(&nNSS.queryPoint,lsq);
		}

        cell.points->setPointScalarValue(i,d);
	}

	return true;
}

CCVector3 GeometricalAnalysisTools::computeGravityCenter(GenericCloud* theCloud)
{
	assert(theCloud);
	unsigned count = theCloud->size();
	if (count)
	{
		double Xsum=0.0;
		double Ysum=0.0;
		double Zsum=0.0;
		const CCVector3 *P;
        theCloud->placeIteratorAtBegining();
        while ((P=theCloud->getNextPoint()))
        {
            Xsum += double(P->x);
            Ysum += double(P->y);
            Zsum += double(P->z);
        }

        return CCVector3( (PointCoordinateType) (Xsum / (double)count),
						  (PointCoordinateType) (Ysum / (double)count),
						  (PointCoordinateType) (Zsum / (double)count));
	}

	return CCVector3();
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
	double mXX=0;
	double mYY=0;
	double mZZ=0;
	double mXY=0;
	double mXZ=0;
	double mYZ=0;

	theCloud->placeIteratorAtBegining();
	for (unsigned i=0;i<n;++i)
	{
		const CCVector3* Q = theCloud->getNextPoint();

        CCVector3 P = *Q-G;
		mXX += double(P.x*P.x);
		mYY += double(P.y*P.y);
		mZZ += double(P.z*P.z);
		mXY += double(P.x*P.y);
		mXZ += double(P.x*P.z);
		mYZ += double(P.y*P.z);
	}

	covMat.m_values[0][0] = mXX/(double)n;
	covMat.m_values[0][0] = mYY/(double)n;
	covMat.m_values[0][0] = mZZ/(double)n;
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY/(double)n;
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ/(double)n;
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ/(double)n;

	return covMat;
}

CCLib::SquareMatrixd GeometricalAnalysisTools::computeCrossCovarianceMatrix(GenericCloud* P,
																			GenericCloud* Q,
																			const PointCoordinateType* pGravityCenter,
																			const PointCoordinateType* qGravityCenter)
{
    assert(P && Q);
	assert(Q->size() == P->size());

	CCVector3 Gp = (pGravityCenter ? CCVector3(pGravityCenter) : computeGravityCenter(P));
	CCVector3 Gq = (qGravityCenter ? CCVector3(qGravityCenter) : computeGravityCenter(Q));

	//shortcuts to output matrix lines
	CCLib::SquareMatrixd covMat(3);
	double* l1 = covMat.line(0);
	double* l2 = covMat.line(1);
	double* l3 = covMat.line(2);

	P->placeIteratorAtBegining();
	Q->placeIteratorAtBegining();

	//sums
	unsigned i,count = P->size();
	for (i=0;i<count;i++)
	{
		CCVector3 Pt = *P->getNextPoint()-Gp;
		CCVector3 Qt = *Q->getNextPoint()-Gq;

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

	covMat.scale(1.0/(double)count);

	return covMat;
}

CCLib::SquareMatrixd GeometricalAnalysisTools::computeWeightedCrossCovarianceMatrix(GenericCloud* P,
																					GenericCloud* Q,
																					const PointCoordinateType* pGravityCenter/*=0*/,
																					const PointCoordinateType* qGravityCenter/*=0*/,
																					ScalarField* weightsP/*=0*/,
																					ScalarField* weightsQ/*=0*/)
{
    assert(P && Q);
	assert(Q->size() == P->size());
	assert(!weightsP || weightsP->currentSize() == P->size());
	assert(!weightsQ || weightsQ->currentSize() == P->size());

	CCVector3 Gp = (pGravityCenter ? CCVector3(pGravityCenter) : computeGravityCenter(P));
	CCVector3 Gq = (qGravityCenter ? CCVector3(qGravityCenter) : computeGravityCenter(Q));

	//shortcuts to output matrix lines
	CCLib::SquareMatrixd covMat(3);
	double* l1 = covMat.line(0);
	double* l2 = covMat.line(1);
	double* l3 = covMat.line(2);

	P->placeIteratorAtBegining();
	Q->placeIteratorAtBegining();

	bool wpIsPositive = (weightsP ? weightsP->isPositive() : false);
	bool wqIsPositive = (weightsQ ? weightsQ->isPositive() : false);

	//sums
	unsigned i,count = P->size();
	DistanceType wp=(DistanceType)1,wq=(DistanceType)1;
	double wSum = 0.0;
	for (i=0;i<count;i++)
	{
		CCVector3 Pt = *P->getNextPoint()-Gp;
		CCVector3 Qt = *Q->getNextPoint()-Gq;

		if (weightsP)
		{
			wp = weightsP->getValue(i);
			if (wpIsPositive)
			{
				if (wp<0.0)
					continue;
			}
			else
			{
				if (wp == BIG_VALUE)
					continue;
			}
		}
		if (weightsQ)
		{
			wq = weightsQ->getValue(i);
			if (wqIsPositive)
			{
				if (wq<0.0)
					continue;
			}
			else
			{
				if (wq == BIG_VALUE)
					continue;
			}
		}
		DistanceType wpq = wp*wq;
		Pt *= wpq;
		wSum += (double)wpq;

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

	if (wSum!=0.0)
		covMat.scale(1.0/wSum);

	return covMat;
}
