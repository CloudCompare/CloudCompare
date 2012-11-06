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

#include "DistanceComputationTools.h"

#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "DgmOctreeReferenceCloud.h"
#include "ReferenceCloud.h"
#include "Neighbourhood.h"
#include "GenericTriangle.h"
#include "GenericIndexedMesh.h"
#include "GenericProgressCallback.h"
#include "ChamferDistanceTransform.h"
#include "FastMarchingForPropagation.h"
#include "ScalarFieldTools.h"
#include "CCMiscTools.h"
#include "CCConst.h"
#include "LocalModel.h"
#include "SimpleTriangle.h"

#include <assert.h>
#include <stdlib.h>

using namespace CCLib;

namespace CCLib
{

	//! Internal structure used by DistanceComputationTools::computePointCloud2MeshDistance
	struct facesInCell
	{
	public:
		//! cell code
		unsigned cellCode;
		//! Indexes of all faces inside cell
		unsigned* faceIndexes;
		//! Number of faces
		unsigned faceCount;

		//! Default constructor
		facesInCell()
			: cellCode(0)
			, faceIndexes(0)
			, faceCount(0)
			, faceIndexesSize(0)
		{
		}

		//! Add a face index to 'faceIndexes'
		bool push(unsigned faceIndex)
		{
			if (faceCount == faceIndexesSize)
			{
				faceIndexesSize += (faceIndexesSize>0 ? faceIndexesSize : 1);
				faceIndexes = (unsigned*)realloc((void*)faceIndexes,sizeof(unsigned)*faceIndexesSize);
				if (!faceIndexes)
					return false;
			}
			faceIndexes[faceCount++]=faceIndex;
			return true;
		}

		//! Destructor
		~facesInCell()
		{
			if (faceIndexes)
				delete[] faceIndexes;
		}

	protected:

		//! Capacity of 'faceIndexes'
		unsigned faceIndexesSize;
	};

	//! Pointer on a facesInCell structure
	typedef facesInCell* facesInCellPtr;

	//! Internal structure used by DistanceComputationTools::computePointCloud2MeshDistance
	struct OctreeAndMeshIntersection
	{
	public:

		//! Octree structure
		DgmOctree* theOctree;
		//! Mesh
		GenericIndexedMesh* theMesh;
		//! Distance transform
		ChamferDistanceTransform* distanceTransform;
		//! Grid occupancy of mesh (minimum indexes)
		int minFillIndexes[3];
		//! Grid occupancy of mesh (maximum indexes)
		int maxFillIndexes[3];

		//! Array of facesInCellPtr structures
		facesInCellPtr **tab;
		//! Element access accelerator (line width)
		unsigned dec;
		//! Number of elements in a slice
		unsigned sliceSize;
		//! Number of slices
		unsigned sliceNumber;

		//! Default constructor
		OctreeAndMeshIntersection()
			: theOctree(0)
			, theMesh(0)
			, distanceTransform(0)
			, tab(0)
			, dec(0)
			, sliceSize(0)
			, sliceNumber(0)
		{
		}

		//! Destructor
		~OctreeAndMeshIntersection()
		{
			if (tab)
			{
				//suppression de tableau d'intersection
				for (unsigned i=0;i<sliceNumber;++i)
				{
					facesInCellPtr* _t = tab[i];
					if (_t)
					{
						for (unsigned j=0;j<sliceSize;++j)
						{
							if (*_t)
								delete *_t;
							++_t;
						}
						delete[] tab[i];
					}
				}

				delete[] tab;
				tab=0;
			}

			if (distanceTransform)
			{
				delete distanceTransform;
				distanceTransform=0;
			}
		}

	};
}

int DistanceComputationTools::computeHausdorffDistance(GenericIndexedCloudPersist* comparedCloud,
														GenericIndexedCloudPersist* referenceCloud,
														Cloud2CloudDistanceComputationParams& params,
														GenericProgressCallback* progressCb/*=0*/,
														DgmOctree* compOctree/*=0*/,
														DgmOctree* refOctree/*=0*/)
{
    assert(comparedCloud && referenceCloud);

	//we spatially 'synchronize' the octrees
	DgmOctree *comparedOctree=compOctree,*referenceOctree=refOctree;
	if (!synchronizeOctrees(comparedCloud,referenceCloud,comparedOctree,referenceOctree, progressCb))
		return -1;

    //if necessary we try to guess the best octree level for distances computation
	if (params.octreeLevel==0)
		params.octreeLevel = comparedOctree->findBestLevelForComparisonWithOctree(referenceOctree);

    //we 'enable' a scalar field  (if it is not already done) to store resulting distances
	comparedCloud->enableScalarField();
	//DGM: we don't reset the previous distances anymore, as they may have been approximated
	//and we only want to update them (or part of them)
	//comparedCloud->forEach(ScalarFieldTools::razDistsToOutValue);

	//we can't use a 'max search distance' criterion if the "Closest Point Set" is requested
	DistanceType maxSearchSquareDist = (params.CPSet || params.maxSearchDist<0.0 ? (DistanceType)-1.0 : params.maxSearchDist*params.maxSearchDist);

	if (params.CPSet)
	{
        if (!params.CPSet->resize(comparedCloud->size()))
		{
			if (!compOctree)
				delete comparedOctree;
			if (!refOctree)
				delete referenceOctree;
			return -3;
		}
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[4];
	additionalParameters[0] = (void*)referenceCloud;
	additionalParameters[1] = (void*)referenceOctree;
	additionalParameters[2] = (void*)&params;
	additionalParameters[3] = (void*)&maxSearchSquareDist;

	int result = 0;

	bool success=false;
#ifdef ENABLE_CLOUD2MESH_DIST_MT
	if (params.multiThread)
	{
		success = (comparedOctree->executeFunctionForAllCellsAtLevel_MT(params.octreeLevel,
																		params.localModel == NO_MODEL ? computeCellHausdorffDistance : computeCellHausdorffDistanceWithLocalModel,
																		additionalParameters,
																		progressCb,
																		"Cloud-Cloud Distance [MT]")!=0);
	}
	else
#endif
	{
		success = (comparedOctree->executeFunctionForAllCellsAtLevel(params.octreeLevel,
																		params.localModel == NO_MODEL ? computeCellHausdorffDistance : computeCellHausdorffDistanceWithLocalModel,
																		additionalParameters,
																		progressCb,
																		"Cloud-Cloud Distance")!=0);
	}

	if (!success)
	{
		//something went wrong
		result = -2;
	}

	if (!compOctree)
        delete comparedOctree;
	if (!refOctree)
        delete referenceOctree;

	return result;
}

DistanceType DistanceComputationTools::computeMeanDist(GenericCloud* theCloud, bool includeNegValues /*= false*/)
{
	theCloud->placeIteratorAtBegining();
	double meanDist = 0.0;
	unsigned i,count=0;

	for (i=0;i<theCloud->size();++i)
	{
		DistanceType V = theCloud->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
		{
			meanDist += (double)V;
			++count;
		}
	}

	return (count>0 ? (DistanceType)(meanDist/(double)count) : 0.0f);
}

DistanceType DistanceComputationTools::computeMeanSquareDist(GenericCloud* theCloud, bool includeNegValues /*= false*/)
{
	theCloud->placeIteratorAtBegining();
	double meanDist = 0.0;
	DistanceType V;
	unsigned i,count=0;

	for (i=0;i<theCloud->size();++i)
	{
		V = theCloud->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
		{
			meanDist += (double)(V*V);
			++count;
		}
	}

	return (DistanceType)(count>0 ? (DistanceType)(meanDist/(double)count) : 0.0f);
}

bool DistanceComputationTools::synchronizeOctrees(GenericIndexedCloudPersist* comparedCloud, GenericIndexedCloudPersist* referenceCloud, DgmOctree* &comparedOctree, DgmOctree* &referenceOctree, GenericProgressCallback* progressCb)
{
    assert(comparedCloud && referenceCloud);

	unsigned nA = comparedCloud->size();
	unsigned nB = referenceCloud->size();

	if (nA*nB==0)
        return false;

	//we compute the bounding box of BOTH clouds
	CCVector3 minsA,minsB,maxsA,maxsB;
	comparedCloud->getBoundingBox(minsA.u,maxsA.u);
	referenceCloud->getBoundingBox(minsB.u,maxsB.u);

	CCVector3 maxD,minD,minPoints,maxPoints;
	uchar k;
	for (k=0;k<3;k++)
	{
		maxPoints.u[k] = maxD.u[k] = ccMax(maxsA.u[k],maxsB.u[k]);
		minPoints.u[k] = minD.u[k] = ccMin(minsA.u[k],minsB.u[k]);
	}

	//we make this bounding-box cubical (+1% growth)
	CCMiscTools::makeMinAndMaxCubical(minD,maxD,0.01);

	//then we (re)compute octree A if necessary
	bool needToRecalculateOctreeA = true;
	if (comparedOctree)
	{
		needToRecalculateOctreeA = false;
		for (k=0;k<3;k++)
			if ((maxD.u[k]!=comparedOctree->getOctreeMaxs().u[k])||(minD.u[k]!=comparedOctree->getOctreeMins().u[k]))
			{
				needToRecalculateOctreeA = true;
				break;
			}
	}

	if (needToRecalculateOctreeA)
	{
		if (comparedOctree)
            comparedOctree->clear();
		else
            comparedOctree = new DgmOctree(comparedCloud);

		if (comparedOctree->build(minD,maxD,&minPoints,&maxPoints,progressCb)<1)
		{
			comparedOctree->clear();
			return false;
		}
	}

	//and we (re)compute octree B as well if necessary
	bool needToRecalculateOctreeB = true;
	if (referenceOctree)
	{
		needToRecalculateOctreeB = false;
		for (k=0;k<3;k++)
			if ((maxD.u[k]!=referenceOctree->getOctreeMins().u[k])||(minD.u[k]!=referenceOctree->getOctreeMaxs().u[k]))
			{
				needToRecalculateOctreeB = true;
				break;
			}
	}

	if (needToRecalculateOctreeB)
	{
		if (referenceOctree)
            referenceOctree->clear();
		else
            referenceOctree = new DgmOctree(referenceCloud);

		if (referenceOctree->build(minD,maxD,&minPoints,&maxPoints,progressCb)<1)
		{
			if (needToRecalculateOctreeA)
                comparedOctree->clear();
			referenceOctree->clear();
			return false;
		}
	}

	//we check that both octrees are ok
	assert(comparedOctree && referenceOctree);
	return (comparedOctree->getNumberOfProjectedPoints()*referenceOctree->getNumberOfProjectedPoints()>0);
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistanceComputationParams*): parameters
// [3] -> (DistanceType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistance(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//additional parameters
	const GenericIndexedCloudPersist* referenceCloud	= (GenericIndexedCloudPersist*)additionalParameters[0];
	const DgmOctree* referenceOctree					= (DgmOctree*)additionalParameters[1];
	Cloud2CloudDistanceComputationParams* params		= (Cloud2CloudDistanceComputationParams*)additionalParameters[2];
	const DistanceType* maxSearchSquareDist		        = (DistanceType*)additionalParameters[3];

	//structure for the nearest neighbor seach
	DgmOctree::NearestNeighboursSearchStruct nPSS;
	nPSS.level								= cell.level;
	nPSS.truncatedCellCode					= cell.truncatedCode;
	nPSS.alreadyVisitedNeighbourhoodSize	= 0;
	nPSS.theNearestPointIndex				= 0;
	nPSS.maxSearchSquareDist				= *maxSearchSquareDist;

	//we already compute the position of the 'equivalent' cell in the refrence octree
	referenceOctree->getCellPos(cell.truncatedCode,cell.level,nPSS.cellPos,true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nPSS.cellPos,cell.level,nPSS.cellCenter);

	//for each point of the current cell (compared octree) we look its nearest neighbour in the reference cloud
	unsigned i,pointCount = cell.points->size();
	for (i=0;i<pointCount;i++)
	{
		cell.points->getPoint(i,nPSS.queryPoint);

		switch (referenceCloud->testVisibility(nPSS.queryPoint))
		{
			case ALL:
			case OUT_OF_RANGE:
			case OUT_OF_FOV:
			case HIDDEN:
				if (!params->CPSet)
				{
					cell.points->setPointScalarValue(i,HIDDEN_VALUE);
					break;
				}
				//otherwise we must treat the point to build up the closest point set!
			case VIEWED:
			{
				DistanceType dist = referenceOctree->findTheNearestNeighborStartingFromCell(nPSS,false);
				if (dist >= 0)
                    dist = sqrt(dist);
                else if (nPSS.maxSearchSquareDist > 0)
                    dist = sqrt(nPSS.maxSearchSquareDist);

				cell.points->setPointScalarValue(i,dist);

				if (params->CPSet)
                    params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i),nPSS.theNearestPointIndex);

				break;
			}
		}
	}

	return true;
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistanceComputationParams*): parameters
// [3] -> (DistanceType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistanceWithLocalModel(const DgmOctree::octreeCell& cell,
                                                                          void** additionalParameters)
{
	//additional parameters
	GenericIndexedCloudPersist* referenceCloud		= (GenericIndexedCloudPersist*)additionalParameters[0];
	const DgmOctree* referenceOctree				= (DgmOctree*)additionalParameters[1];
	Cloud2CloudDistanceComputationParams* params	= (Cloud2CloudDistanceComputationParams*)additionalParameters[2];
	const DistanceType* maxSearchSquareDist			= (DistanceType*)additionalParameters[3];

	assert(params && params->localModel != NO_MODEL);

	//structure for the nearest neighbor seach
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.truncatedCellCode					= cell.truncatedCode;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDist				= *maxSearchSquareDist;
	//we already compute the position of the 'equivalent' cell in the refrence octree
	referenceOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//structures for determining the nearest neighbours of the 'nearest neighbour' (to compute the local model)
	//either inside a sphere or the k nearests
	DgmOctree::NearestNeighboursSearchStruct nNSS_Model_kNN;
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS_Model_spherical;
	if (!params->useSphericalSearchForLocalModel)
	{
		nNSS_Model_kNN.level								= cell.level;
		nNSS_Model_kNN.alreadyVisitedNeighbourhoodSize		= 0;
		nNSS_Model_kNN.truncatedCellCode					= cell.truncatedCode;
		nNSS_Model_kNN.minNumberOfNeighbors					= params->kNNForLocalModel;
		nNSS_Model_kNN.maxSearchSquareDist					= -1.0;
		memcpy(nNSS_Model_kNN.cellCenter,nNSS.cellCenter,3*sizeof(PointCoordinateType));
		memcpy(nNSS_Model_kNN.cellPos,nNSS.cellPos,3*sizeof(int));
	}
	else
	{
		nNSS_Model_spherical.level						= cell.level;
		nNSS_Model_spherical.truncatedCellCode			= cell.truncatedCode;
		nNSS_Model_spherical.prepare(params->radiusForLocalModel,cell.parentOctree->getCellSize(nNSS_Model_spherical.level));
		memcpy(nNSS_Model_spherical.cellCenter,nNSS.cellCenter,3*sizeof(PointCoordinateType));
		memcpy(nNSS_Model_spherical.cellPos,nNSS.cellPos,3*sizeof(int));
	}

	//already computed models
	std::vector<LocalModel*> models;

	//for each point of the current cell (compared octree) we look its nearest neighbour in the reference cloud
	unsigned i,pointCount=cell.points->size();
	for (i=0;i<pointCount;++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		switch (referenceCloud->testVisibility(nNSS.queryPoint))
		{
			case ALL:
			case OUT_OF_FOV:
			case OUT_OF_RANGE:
			case HIDDEN:
				if (!params->CPSet)
				{
					cell.points->setPointScalarValue(i,HIDDEN_VALUE);
					break;
				}
				//otherwise we must treat the point to build up the closest point set!
			case VIEWED:
			{
				//first, we look for the nearest point to "_queryPoint" in the reference cloud
				DistanceType squareDistPt = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS,false);
				DistanceType distPt = -1.0;

				//if it exists
				if (squareDistPt >= 0)
				{
					distPt = sqrt(squareDistPt);

					CCVector3 nearestPoint;
					referenceCloud->getPoint(nNSS.theNearestPointIndex,nearestPoint);

					//local model for the 'nearest point'
					LocalModel* lm = 0;

					if (params->reuseExistingLocalModels)
					{
						//we look if the nearest point is close to existing models
						for (std::vector<LocalModel*>::const_iterator it = models.begin(); it!=models.end(); ++it)
						{
							//we take the first model that 'includes' the nearest point
							if ( ((*it)->getCenter() - nearestPoint).norm2() <= (*it)->getSquareSize())
							{
								lm=*it;
								break;
							}
						}
					}

					if (!lm)
					{
						DgmOctree::NeighboursSet* neighbours = 0;

						//let's grab the nearest neighbours of the 'nearest point'
						unsigned kNN = 0;
						if (params->useSphericalSearchForLocalModel)
						{
							nNSS_Model_spherical.queryPoint = nearestPoint;
							neighbours = &nNSS_Model_spherical.pointsInNeighbourhood;
							//we only need to sort neighbours if we want to use the 'reuseExistingLocalModels' optimization
							kNN = referenceOctree->findNeighborsInASphereStartingFromCell(nNSS_Model_spherical,params->radiusForLocalModel,params->reuseExistingLocalModels);
						}
						else
						{
							nNSS_Model_kNN.queryPoint = nearestPoint;
							neighbours = &nNSS_Model_kNN.pointsInNeighbourhood;
							kNN = referenceOctree->findNearestNeighborsStartingFromCell(nNSS_Model_kNN);
							kNN = ccMin(kNN,params->kNNForLocalModel);
						}

						//if there's enough neighbours
						if (kNN >= CC_LOCAL_MODEL_MIN_SIZE[params->localModel])
						{
							assert(neighbours);

							DgmOctreeReferenceCloud neighboursCloud(neighbours,kNN);
							Neighbourhood Z(&neighboursCloud);

							//Neighbours are sorted, so the farthest is at the end. It also gives us
							//an approximation of the model 'size'
							DistanceType maxSquareDist = -1.0;
							
							if (params->reuseExistingLocalModels)
								maxSquareDist = neighbours->at(kNN-1).squareDist;

							lm = new LocalModel(Z,params->localModel,nearestPoint,maxSquareDist);

							if (params->reuseExistingLocalModels)
								//we add the model to the 'existing models' list
								models.push_back(lm);

							//neighbours->clear();
						}
					}

					//if we have a local model
					if (lm)
					{
						DistanceType distToModel = lm->computeDistanceFromModelToPoint(&nNSS.queryPoint);
						//if (distToModel<distPt)
                            distPt=distToModel;

						if (!params->reuseExistingLocalModels)
						{
							//we don't need the local model anymore!
							delete lm;
							lm=0;
						}
					}
				}
                else if (nNSS.maxSearchSquareDist > 0)
                {
                    distPt = sqrt(nNSS.maxSearchSquareDist);
                }

				cell.points->setPointScalarValue(i,distPt);

				if (params->CPSet)
                    params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i),nNSS.theNearestPointIndex);

				break;
			}
		}

		cell.points->forwardIterator();
	}

	//clear all models for this cell
	while (!models.empty())
	{
		delete models.back();
		models.pop_back();
	}

	return true;
}


/****** CALCUL DE DISTANCES ENTRE UN NUAGE ET UN MAILLAGE ******/

//Internal structure used by DistanceComputationTools::computePointCloud2MeshDistance
struct cellToTest
{
#ifdef OCTREE_CODES_64_BITS
	typedef int Type;
#else
	typedef short Type; //we use shorts to save memory!
#endif
	//! Cell position
	Type pos[3];
	//! Cell size
	Type cellSize;
	//! Subdivision level
	uchar level;
};

//Calcul de l'intersection entre un octree et un maillage
//(fonction utile pour "computePointCloud2MeshDistance")
int DistanceComputationTools::intersectMeshWithOctree(OctreeAndMeshIntersection* theIntersection, uchar octreeLevel, GenericProgressCallback* progressCb)
{
	assert(theIntersection);
	DgmOctree* theOctree = theIntersection->theOctree;
	GenericIndexedMesh* theMesh = theIntersection->theMesh;

	//variables utiles
	int cellPos[9],maxPos[3],minPos[3],delta[3];
	const CCVector3* triPoints[3]; //sommets du triangle en cours
	char pointsPosition[27]; //position des sommets des cubes par rapport au plan du triangle en cours

	//Taille des cellules
	PointCoordinateType cellLength = theIntersection->theOctree->getCellSize(octreeLevel); //taille d'une cellule élémentaire
	std::vector<cellToTest> cellsToTest;

#define OPTIMIZE_STD_VECTOR_USAGE
#ifdef OPTIMIZE_STD_VECTOR_USAGE
	unsigned cellsToTestCapacity=1;
	cellsToTest.resize(cellsToTestCapacity);
	unsigned cellsToTestCount=0;
#else
	cellToTest currentCell;
	cellToTest* _currentCell = &currentCell;
#endif

	//on récupère les limites de l'octree
	const CCVector3& minBB = theOctree->getOctreeMins();

	//le nombre de triangles
	unsigned numberOfTriangles = theMesh->size();

	//barre de progression
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfTriangles);
		char buffer[64];
		sprintf(buffer,"Faces = %i",numberOfTriangles);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle("Intersect Cloud/Mesh");
		progressCb->start();
	}
	//*/

	//Pour chaque triangle, il faut chercher les cellules intersectées
	theMesh->placeIteratorAtBegining();
	for (unsigned n=0;n<numberOfTriangles;++n)
	{
		//on extrait la position (en terme de cellules) de chaque sommet du triangle
		const GenericTriangle* T = theMesh->_getNextTriangle();
		triPoints[0] = T->_getA();
		triPoints[1] = T->_getB();
		triPoints[2] = T->_getC();

		CCVector3 AB = (*triPoints[1])-(*triPoints[0]);
		CCVector3 BC = (*triPoints[2])-(*triPoints[1]);
		CCVector3 CA = (*triPoints[0])-(*triPoints[2]);

		//on teste la validité du triangle pour éviter des GROS problèmes plus tard
		if ( AB.norm2() > ZERO_TOLERANCE &&
             BC.norm2() > ZERO_TOLERANCE &&
             CA.norm2() > ZERO_TOLERANCE)
		{
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[0],cellPos,octreeLevel);
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[1],cellPos+3,octreeLevel);
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[2],cellPos+6,octreeLevel);

			//on cherche les dimensions maximales
			for (int k=0;k<3;k++)
			{
				int i=ccMin(cellPos[3+k],cellPos[6+k]);
				minPos[k] = ccMin(cellPos[k],i);
				i=ccMax(cellPos[3+k],cellPos[6+k]);
				maxPos[k] = ccMax(cellPos[k],i);
			}

#ifdef OPTIMIZE_STD_VECTOR_USAGE
			//first cell
			assert(cellsToTestCapacity>0);
			cellsToTestCount=1;
			cellToTest* _currentCell = &cellsToTest[0/*cellsToTestCount-1*/];
#endif

            for (int k=0;k<3;k++)
			{
				delta[k] = maxPos[k]-minPos[k]+1;
				_currentCell->pos[k] = (cellToTest::Type)minPos[k]; //par défaut, c'est là que commence la cellule
			}

			CCVector3 distanceToOctreeMinBorder = minBB-(*triPoints[0]);

			//on calcule la normale à la face, "N"
			CCVector3 N = AB.cross(BC);

			//distance maximale entre les 3 cellules des sommets
			int maxSize = ccMax(delta[0],delta[1]);
			maxSize = ccMax(maxSize,delta[2]);

			//la distance maximale nous indique la taille de la boite englobante minimale et donc le niveau d'octree maximal
			_currentCell->level = octreeLevel-(maxSize>1 ? uchar(ceil(log(double(maxSize))/log(2.0))) : 0);
			_currentCell->cellSize = 1 << (octreeLevel-_currentCell->level);

			//on obtient normalement la cellule cubique qui englobe complètement le triangle
			//ATTENTION:
			//- CE N'EST PAS FORCEMENT UNE CELLULE DE L'OCTREE PUISQUE SA POSITION EST LIBRE, ET POTENTIELLEMENT IMPAIRE)
			//- IL EST POSSIBLE QUE currentCell.pos[k]+currentCell.cellSize > octreeLength !
#ifndef OPTIMIZE_STD_VECTOR_USAGE
			cellsToTest.push_back(currentCell);
#endif

			//on peut maintenant tenter de detecter les cellules qui intersectent la face par récurrence
#ifndef OPTIMIZE_STD_VECTOR_USAGE
			while (!cellsToTest.empty())
#else
			while (cellsToTestCount>0)
#endif
			{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
				currentCell = cellsToTest.back();
				cellsToTest.pop_back();
#else
				_currentCell = &cellsToTest[--cellsToTestCount];
#endif

				//si on est au dernier niveau de l'octree, on peut mettre à jour les cases de la grille 3D
				if (_currentCell->level==octreeLevel)
				{
					//la position (absolue) du centre de la cellule
					theOctree->computeCellCenter(_currentCell->pos,octreeLevel,AB.u);

					//on verifie que le triangle intersecte effectivement la boite
					if (CCMiscTools::triBoxOverlap(AB.u,cellLength*0.5f,triPoints))
					{
						if (((int)_currentCell->pos[0]>=theIntersection->minFillIndexes[0] && (int)_currentCell->pos[0]<=theIntersection->maxFillIndexes[0])&&
							((int)_currentCell->pos[1]>=theIntersection->minFillIndexes[1] && (int)_currentCell->pos[1]<=theIntersection->maxFillIndexes[1])&&
							((int)_currentCell->pos[2]>=theIntersection->minFillIndexes[2] && (int)_currentCell->pos[2]<=theIntersection->maxFillIndexes[2]))
						{
                            if (theIntersection->tab)
							{
								int index = ((int)_currentCell->pos[1]-theIntersection->minFillIndexes[1])*theIntersection->dec +
                                            ((int)_currentCell->pos[2]-theIntersection->minFillIndexes[2]);
								facesInCellPtr* f = theIntersection->tab[(int)_currentCell->pos[0]-theIntersection->minFillIndexes[0]]+index;

								if (!(*f))
								{
									(*f) = new facesInCell();
									(*f)->cellCode = theOctree->generateTruncatedCellCode(_currentCell->pos,octreeLevel);
								}

								//on rajoute le triangle en cours à la liste des triangles secants
								(*f)->push(n);
							}

                            if (theIntersection->distanceTransform)
                            {
                                theIntersection->distanceTransform->setZero((int)_currentCell->pos[0]-theIntersection->minFillIndexes[0],
                                                                            (int)_currentCell->pos[1]-theIntersection->minFillIndexes[1],
                                                                                (int)_currentCell->pos[2]-theIntersection->minFillIndexes[2]);
                            }
						}
					}
				}
				else
				{
					int halfCellSize = _currentCell->cellSize>>1;

					//on calcule la position des bords des cubes (il y en a 3*3*3=27) par rapport à la face
					{
						char* _pointsPosition = pointsPosition;
						for (int i=0;i<3;++i)
						{
							//la position relative des sommets des 4 sous-cellules
							AB.x = distanceToOctreeMinBorder.x+PointCoordinateType((int)_currentCell->pos[0]+i*halfCellSize)*cellLength;
							for (int j=0;j<3;++j)
							{
								AB.y = distanceToOctreeMinBorder.y+PointCoordinateType((int)_currentCell->pos[1]+j*halfCellSize)*cellLength;
								for (int k=0;k<3;++k)
								{
									AB.z = distanceToOctreeMinBorder.z+PointCoordinateType((int)_currentCell->pos[2]+k*halfCellSize)*cellLength;

									//on regarde de quel côté de la face est le point
									*_pointsPosition++/*pointsPosition[i*9+j*3+k]*/ = (AB.dot(N) < 0.0 ? -1 : 1);
								}
							}
						}
					}

#ifndef OPTIMIZE_STD_VECTOR_USAGE
					cellToTest newCell;
					newCell.level = currentCell.level+1;
					newCell.cellSize = halfCellSize;
					cellToTest* _newCell=&newCell;
#else
					//the first new cell will be written over the actual one
					//so we need to remember its position!
					int currentCellPos[3]={_currentCell->pos[0],_currentCell->pos[1],_currentCell->pos[2]};

					//if necessary we enlarge the queue
					if (cellsToTestCount+27>cellsToTestCapacity)
					{
						cellsToTestCapacity = std::max(cellsToTestCapacity+27,2*cellsToTestCapacity);
						try
						{
							cellsToTest.resize(cellsToTestCapacity);
						}
						catch (.../*const std::bad_alloc&*/) //out of memory
						{
							return -1;
						}
					}

					//the first new cell will be written over the actual one
					cellToTest* _newCell = &cellsToTest[cellsToTestCount];
					_newCell->level++;
					_newCell->cellSize = halfCellSize;
#endif

					//on regarde la "position" de chacun des 8 sous-cubes
					for (int i=0;i<2;++i)
					{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
						_newCell->pos[0]=(int)currentCell.pos[0]+i*halfCellSize;
#else
						_newCell->pos[0]=(int)currentCellPos[0]+i*halfCellSize;
#endif
						//on test rapidement pour savoir si le cube est au moins en intersection avec
						//la boite englobante du triangle
						if (((int)_newCell->pos[0]+halfCellSize>=minPos[0])&&((int)_newCell->pos[0]<=maxPos[0]))
						for (int j=0;j<2;++j)
						{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
							_newCell->pos[1]=(int)currentCell.pos[1]+j*halfCellSize;
#else
							_newCell->pos[1]=(int)currentCellPos[1]+j*halfCellSize;
#endif
							if (((int)_newCell->pos[1]+halfCellSize>=minPos[1])&&((int)_newCell->pos[1]<=maxPos[1]))
							for (int k=0;k<2;++k)
							{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
								_newCell->pos[2]=(int)currentCell.pos[2]+k*halfCellSize;
#else
								_newCell->pos[2]=(int)currentCellPos[2]+k*halfCellSize;
#endif
								if (((int)_newCell->pos[2]+halfCellSize>=minPos[2])&&((int)_newCell->pos[2]<=maxPos[2]))
								{
									const char* _pointsPosition = pointsPosition+(i*9+j*3+k);
									char sum = _pointsPosition[0]+_pointsPosition[1]+_pointsPosition[3]+
												_pointsPosition[4]+_pointsPosition[9]+_pointsPosition[10]+
												_pointsPosition[12]+_pointsPosition[13];

									//si tout les points ne sont pas du même côté alors la face intersecte potentiellement le cube
									if (abs(sum)<8)
									{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
                                        cellsToTest.push_back(newCell);
#else
										++cellsToTestCount;
										//we make newCell point on next cell in array (we copy current info by the way)
										cellsToTest[cellsToTestCount] = *_newCell;
										_newCell = &cellsToTest[cellsToTestCount];
#endif
									}
								}
							}
						}
					}
				}
			}
		}

		//system("PAUSE");

		if (nProgress && !nProgress->oneStep())
		{
			delete nProgress;
			return -1;
		}
	}

	if (nProgress)
		delete nProgress;
	nProgress=0;

	return 0;
}


//Calcul de la distance au carré entre un maillage et un nuage, accéléré par la structure "OctreeAndMeshIntersection"
int DistanceComputationTools::computePointCloud2MeshDistanceWithOctree(OctreeAndMeshIntersection* theIntersection,
                                                                                uchar octreeLevel,
																				bool signedDistances,
																				bool flipTriangleNormals/*=false*/,
                                                                                DistanceType maxSearchDist/*=-1.0*/,
                                                                                GenericProgressCallback* progressCb/*=0*/)
{
	assert(theIntersection);
	assert(!signedDistances || !theIntersection->distanceTransform); //signed distances are not compatible with Distance Transform acceleration

	DgmOctree* theOctree = theIntersection->theOctree;
	GenericIndexedMesh* theMesh = theIntersection->theMesh;

	//taille d'une cellule d'octree
	PointCoordinateType cellLength = theOctree->getCellSize(octreeLevel);

	//extraction des indexes et codes des cellules du niveau "octreeLevel"
	DgmOctree::cellsContainer cellCodesAndIndexes;
	theOctree->getCellCodesAndIndexes(octreeLevel,cellCodesAndIndexes,true);

	unsigned numberOfCells = cellCodesAndIndexes.size();

	//variables utiles
	DgmOctree::cellsContainer::const_iterator pCodeAndIndex = cellCodesAndIndexes.begin();
	ReferenceCloud Yk(theOctree->associatedCloud());

	//bounded search
	bool boundedSearch=(maxSearchDist>=0);
	int maxNeighbourhoodLength=0; //distance maximale de recherche des voisinages, dans le cas où "maxSearchDist" est défini
	DistanceType maxSearchDistSq2=0; //time saver
	if (boundedSearch)
	{
		maxSearchDistSq2=maxSearchDist*maxSearchDist;
		maxNeighbourhoodLength = int(ceil(maxSearchDist/cellLength+(sqrt(2.0f)-1.0f)/2.0f));
	}

	//si toutes les distances doivent être approximées par Chanfrein
	//on va simplement assigner la distance "approximative" calculée au niveau de chaque
	//cellule par propagation aux points inclus dedans
	if (theIntersection->distanceTransform && !boundedSearch)
	{
		//on traite cellule par cellule
		for (unsigned i=0;i<numberOfCells;++i,++pCodeAndIndex)
		{
			theOctree->getPointsInCellByCellIndex(&Yk,pCodeAndIndex->theIndex,octreeLevel);

			//on récupère la position de la cellule (dans startPos)
			int startPos[3];
			theOctree->getCellPos(pCodeAndIndex->theCode,octreeLevel,startPos,true);

			//on exprime maintenant startPos relativement aux bords de la grille
			startPos[0] -= theIntersection->minFillIndexes[0];
			startPos[1] -= theIntersection->minFillIndexes[1];
			startPos[2] -= theIntersection->minFillIndexes[2];

			int dist = theIntersection->distanceTransform->getValue(startPos);

			//on assigne à tous les points la distance de chamfrein "moyenne"
			DistanceType maxRadius = 0.3333f * (DistanceType)dist * (DistanceType)cellLength;

			unsigned count=Yk.size();
			for (unsigned j=0;j<count;++j)
				Yk.setPointScalarValue(j,maxRadius);

			//Yk.clear(); //useless
		}

		return 0;
	}

	//sinon on va comparer les points aux triangles présents au niveau de leur cellules
	//ou de leurs voisines (l'algo METRO standard en fait)

	//Progress callback
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfCells);
		char buffer[256];
		sprintf(buffer,"Cells=%i",numberOfCells);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle(signedDistances ? "Compute signed distances" : "Compute distances");
		progressCb->start();
	}
	//*/

	//variables et structures utiles
	std::vector<unsigned> trianglesToTest;
#ifdef OPTIMIZE_STD_VECTOR_USAGE
	unsigned trianglesToTestCount=0;
	unsigned trianglesToTestCapacity=0;
#endif
	const DistanceType normalSign = (flipTriangleNormals ? -1.0f : 1.0f);

	//nombre de triangles dans le mesh
	unsigned numberOfTriangles = theMesh->size();

	//structure d'accélération
	unsigned* treatedTriangle = new unsigned[numberOfTriangles];
	if (treatedTriangle)
		memset(treatedTriangle,0,sizeof(unsigned)*numberOfTriangles);
	//sinon, c'est pas grave, on peu faire sans

	//tableau des distances minimales (version "persistente" pour économiser la mémoire)
	std::vector<DistanceType> minDists;

	//on traite cellule par cellule
	for (unsigned cellIndex=1;cellIndex<=numberOfCells;++cellIndex,++pCodeAndIndex) //cellIndex = identifiant unique de la cellule en cours
	{
		theOctree->getPointsInCellByCellIndex(&Yk,pCodeAndIndex->theIndex,octreeLevel);

		//on récupère la position de la cellule (dans startPos)
		int startPos[3];
		theOctree->getCellPos(pCodeAndIndex->theCode,octreeLevel,startPos,true);

		//on en déduit le symétrique ainsi que la distance au bord de la grille le plus éloigné (maxDistToBoundaries)
		int maxDistToBoundaries = 0;
		int distToLowerBorder[3],distToUpperBorder[3];
		for (unsigned k=0;k<3;++k)
		{
			distToLowerBorder[k] = startPos[k]-theIntersection->minFillIndexes[k];
			maxDistToBoundaries=ccMax(maxDistToBoundaries,distToLowerBorder[k]);
			distToUpperBorder[k] = theIntersection->maxFillIndexes[k]-startPos[k];
			maxDistToBoundaries=ccMax(maxDistToBoundaries,distToUpperBorder[k]);
		}
		int maxDist = maxDistToBoundaries;

		//on determine son centre
		PointCoordinateType cellCenter[3];
		theOctree->computeCellCenter(startPos,octreeLevel,cellCenter);

		//on exprime maintenant startPos relativement aux bords de la grille
		startPos[0] -= theIntersection->minFillIndexes[0];
		startPos[1] -= theIntersection->minFillIndexes[1];
		startPos[2] -= theIntersection->minFillIndexes[2];

		//initialisation de la récurrence
		DistanceType maxRadius=0;
		int dist=0;

		if (theIntersection->distanceTransform)
		{
			unsigned short dist = theIntersection->distanceTransform->getValue(startPos);
			maxRadius = ((DistanceType)dist * 0.3333f) * cellLength;

			//if (boundedSearch)  //should always be true if we are here!
			{
				maxSearchDistSq2 = ccMax(maxSearchDist,maxRadius);
				maxSearchDistSq2 *= maxSearchDistSq2;
			}
		}

		//on raz la strcture persistente
		//minDists.clear();
		unsigned remainingPoints = Yk.size();
		if (minDists.size()<remainingPoints)
		{
			try
			{
				minDists.resize(remainingPoints);
			}
			catch (.../*const std::bad_alloc&*/) //out of memory
			{
				if (treatedTriangle)
					delete[] treatedTriangle;
				if (nProgress)
					delete nProgress;
				return -1;
			}
		}

		//on calcule pour chaque point sa distance au bord de la cellule la plus proche
		//cela nous permettra de recalculer plus rapidement la distance d'elligibilité
		//du triangle le plus proche
		for (unsigned j=0;j<remainingPoints;++j)
		{
			//coordonnées du point courant
			const CCVector3 *tempPt = Yk.getPointPersistentPtr(j);
			//distance du bord le plus proche = taille de la cellule - distance la plus grande par rapport au centre de la cellule
			//minDists.push_back(cellLength*0.5-DgmOctree::computeMaxDistanceToCellCenter(tempPt,cellCenter));
			minDists[j] = DgmOctree::ComputeMinDistanceToCellBorder(tempPt,cellLength,cellCenter);
		}

		//MODE : ON CALCULE LES DISTANCES PRECISES EN DESSOUS DE "maxSearchDist"
		//ET LES DISTANCES DE CHANFREIN AU DESSUS
		if (boundedSearch)
		{
			//on redéfinit maxDist pour prendre en compte le fait qu'il est inutile d'aller
			//chercher plus loin que "maxNeighbourhoodLength"
			maxDist = ccMin(maxDistToBoundaries,maxNeighbourhoodLength);

			for (unsigned j=0;j<remainingPoints;++j)
				Yk.setPointScalarValue(j,maxSearchDistSq2);
		}

		//on va essayer de trouver les triangles les plus proches de chaque point du "voisinage" Yk
		while (remainingPoints>0 && dist<=maxDist)
		{
			//on va tester les cellules voisines à une distance "dist"
			//a,b,c,d,e,f représentent l'extension spatiale du voisinage INCLUS DANS LA GRILLE 3D
			//selon les 6 directions -X,+X,-Y,+Y,-Z,+Z
			int a = ccMin(dist,distToLowerBorder[0]);
			int b = ccMin(dist,distToUpperBorder[0]);
			int c = ccMin(dist,distToLowerBorder[1]);
			int d = ccMin(dist,distToUpperBorder[1]);
			int e = ccMin(dist,distToLowerBorder[2]);
			int f = ccMin(dist,distToUpperBorder[2]);

			int index0 = startPos[0]-a;
			for (int i=-a;i<=b;i++)
			{
				bool imax = (abs(i)==dist);
				facesInCellPtr *_tab0 = theIntersection->tab[index0];

				//index0 = (startPos[0]=i)*dec2;
				int index = (startPos[1]-c)*(int)theIntersection->dec;
				for (int j=-c;j<=d;j++)
				{
					//index = index0+(startPos[1]+j)*dec1;

					//si i ou j est maximal
					if (imax || abs(j)==dist)
					{
						//on est forcément sur le bord du voisinage
						facesInCellPtr *_tab = _tab0+(index+startPos[2]-e);

						for (int k=-e;k<=f;k++)
						{
							//element correspondant
							if (*_tab)
							{
								const facesInCellPtr& element = *_tab;
#ifdef OPTIMIZE_STD_VECTOR_USAGE
								if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
#endif
								//pour chaque triangle intersectant la cellule en cours
								for (unsigned p=0;p<element->faceCount;++p)
								{
									if (treatedTriangle)
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//si le triangle n'a pas déja été inséré
										if (treatedTriangle[indexTri] != cellIndex)
										{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
											trianglesToTest.push_back(indexTri);
#else
											trianglesToTest[trianglesToTestCount++] = indexTri;
#endif
											treatedTriangle[indexTri] = cellIndex;
										}
									}
									else
									{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
										trianglesToTest.push_back(element->faceIndexes[p]);
#else
										trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
#endif
									}
								}
							}

							++_tab;
						}
					}
					else //on doit se mettre au bord du cube
					{
						if (e==dist) //côté négatif
						{
							//index correspondant
							const facesInCellPtr& element = _tab0[index+startPos[2]-e];

							if (element)
							{
#ifdef OPTIMIZE_STD_VECTOR_USAGE
								if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
#endif
								//pour chaque triangle intersectant la cellule en cours
								for (unsigned p=0;p<element->faceCount;++p)
								{
									if (treatedTriangle)
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//si le triangle n'a pas déja été inséré
										if (treatedTriangle[indexTri] != cellIndex)
										{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
											trianglesToTest.push_back(indexTri);
#else
											trianglesToTest[trianglesToTestCount++] = indexTri;
#endif
											treatedTriangle[indexTri] = cellIndex;
										}
									}
									else
									{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
										trianglesToTest.push_back(element->faceIndexes[p]);
#else
										trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
#endif
									}
								}
							}
						}

						if (f==dist && dist>0) //côté positif
						{
							//index correspondant
							const facesInCellPtr& element = _tab0[index+startPos[2]+f];

							if (element)
							{
#ifdef OPTIMIZE_STD_VECTOR_USAGE
								if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
#endif
								//pour chaque triangle intersectant la cellule en cours
								for (unsigned p=0;p<element->faceCount;++p)
								{
									if (treatedTriangle)
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//si le triangle n'a pas déja été inséré
										if (treatedTriangle[indexTri] != cellIndex)
										{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
											trianglesToTest.push_back(indexTri);
#else
											trianglesToTest[trianglesToTestCount++] = indexTri;
#endif
											treatedTriangle[indexTri] = cellIndex;
										}
									}
									else
									{
#ifndef OPTIMIZE_STD_VECTOR_USAGE
										trianglesToTest.push_back(element->faceIndexes[p]);
#else
										trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
#endif
									}
								}
							}
						}
					} // fin else

					index += (int)theIntersection->dec;
				}

				index0++;
			}

			//pour chaque triangle potentiellement intersectant
			//remarque : normalement on n'a pas déja testé ce triangle
			//pour une distance de voisinage inferieure (grace à treatedTriangle)
#ifndef OPTIMIZE_STD_VECTOR_USAGE
			bool firstComparisonDone = !trianglesToTest.empty();
#else
			bool firstComparisonDone = (trianglesToTestCount!=0);
#endif

			//pour chaque triangle
#ifndef OPTIMIZE_STD_VECTOR_USAGE
			while (!trianglesToTest.empty())
#else
			while (trianglesToTestCount != 0)
#endif
			{
				//on récupère les coordonnées de ses sommets (references)
#ifndef OPTIMIZE_STD_VECTOR_USAGE
				GenericTriangle *tempTri = theMesh->_getTriangle(trianglesToTest.back());
				trianglesToTest.pop_back();
#else
				GenericTriangle *tempTri = theMesh->_getTriangle(trianglesToTest[--trianglesToTestCount]);
#endif

				//pour chaque point dans la cellule
				Yk.placeIteratorAtBegining();
				if (signedDistances)
				{
					for (unsigned j=0;j<remainingPoints;++j)
					{
						//on calcule la distance point/triangle (attention, on travaille avec des distances au carré)
						DistanceType dPTri = /*normalSign**/computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),tempTri,true);
						//si elle est plus petite que la distance actuelle du point, on remplace
						DistanceType min_d = Yk.getCurrentPointScalarValue();
						if (min_d*min_d > dPTri*dPTri)
							Yk.setCurrentPointScalarValue(normalSign*dPTri);
						Yk.forwardIterator();
					}
				}
				else //squared distances
				{
					for (unsigned j=0;j<remainingPoints;++j)
					{
						//on calcule la distance point/triangle (attention, on travaille avec des distances au carré)
						DistanceType dPTri = computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),tempTri,false);
						//si elle est plus petite que la distance actuelle du point, on remplace
						DistanceType min_d = Yk.getCurrentPointScalarValue();
						if (min_d < 0 || dPTri<min_d)
							Yk.setCurrentPointScalarValue(dPTri);
						Yk.forwardIterator();
					}
				}
			}

			//maintenant on retire tous les points "élus" lors de ce tour
			if (firstComparisonDone)
			{
				unsigned j=0;
				while (j<remainingPoints)
				{
					//distance d'elligibilité
					DistanceType elligibleDist = minDists[j]+maxRadius;
					DistanceType dPTri = Yk.getPointScalarValue(j);
					if (signedDistances)
						dPTri*=dPTri;
					if (dPTri <= elligibleDist*elligibleDist)
					{
						//on supprime le point courant
						Yk.removePointGlobalIndex(j);
						//on applique l'opération équivalente (voir ReferenceCloud::removeCurrentPointGlobalIndex)
						//au tableau donnant la distance minimale par rapport à la cellule
						assert(remainingPoints>0);
						--remainingPoints;
						minDists[j] = minDists[remainingPoints];
						//minDists.pop_back();

					}
					else
					{
						++j;
					}
				}
			}
			//*/

			++dist;
			maxRadius += cellLength;
		}

		//Yk.clear(); //pas nécessaire

		if (nProgress && !nProgress->oneStep())
			break;
	}

	if (nProgress)
		delete nProgress;
	nProgress=0;

	if (treatedTriangle)
		delete[] treatedTriangle;
	treatedTriangle=0;

	return 0;
}

#ifdef ENABLE_CLOUD2MESH_DIST_MT

#include <QtCore/QtCore>
#include <QtGui/QApplication>

/*** MULTI THREADING WRAPPER ***/

static DgmOctree* s_octree_MT = 0;
static NormalizedProgress* s_normProgressCb_MT = 0;
static bool s_cellFunc_MT_success = true;
static unsigned char s_octreeLevel_MT = 0;
static OctreeAndMeshIntersection* s_theIntersection_MT = 0;
static bool s_signedDistances_MT = true;
static DistanceType s_normalSign_MT = 1.0f;

//'treatedTriangle' mechanism (based on bit mask)
#include <QtCore/QBitArray>
static std::vector<QBitArray*> s_bitArrayPool_MT;
static bool s_useBitArrays_MT = true;
static QMutex s_currentBitMaskMutex;

void cloudMeshDistCellFunc_MT(const DgmOctree::indexAndCode& desc)
{
	//skip cell if process is aborted/has failed
	if (!s_cellFunc_MT_success)
		return;

	if (s_normProgressCb_MT)
	{
		QApplication::processEvents(); //let the application breath!
		if (!s_normProgressCb_MT->oneStep())
		{
			s_cellFunc_MT_success = false;
            return;
		}
	}

	ReferenceCloud Yk(s_octree_MT->associatedCloud());
	s_octree_MT->getPointsInCellByCellIndex(&Yk,desc.theIndex,s_octreeLevel_MT);

	//tableau des distances minimales
	unsigned remainingPoints = Yk.size();

	DistanceType* minDists = new DistanceType[remainingPoints];
	if (!minDists) //not enough memory?
	{
		s_cellFunc_MT_success = false;
		return;
	}

	//on récupère la position de la cellule (dans startPos)
	int startPos[3];
	s_octree_MT->getCellPos(desc.theCode,s_octreeLevel_MT,startPos,true);

	//on en déduit le symétrique ainsi que la distance au bord de la grille le plus éloigné (maxDistToBoundaries)
	int maxDistToBoundaries = 0;
	int distToLowerBorder[3],distToUpperBorder[3];
	for (unsigned k=0;k<3;++k)
	{
		distToLowerBorder[k] = startPos[k]-s_theIntersection_MT->minFillIndexes[k];
		maxDistToBoundaries=ccMax(maxDistToBoundaries,distToLowerBorder[k]);
		distToUpperBorder[k] = s_theIntersection_MT->maxFillIndexes[k]-startPos[k];
		maxDistToBoundaries=ccMax(maxDistToBoundaries,distToUpperBorder[k]);
	}
	int maxDist = maxDistToBoundaries;

	//on determine son centre
	PointCoordinateType cellCenter[3];
	s_octree_MT->computeCellCenter(startPos,s_octreeLevel_MT,cellCenter);

	//on exprime maintenant startPos relativement aux bords de la grille
	startPos[0] -= s_theIntersection_MT->minFillIndexes[0];
	startPos[1] -= s_theIntersection_MT->minFillIndexes[1];
	startPos[2] -= s_theIntersection_MT->minFillIndexes[2];

	//taille d'une cellule d'octree
	const PointCoordinateType& cellLength = s_octree_MT->getCellSize(s_octreeLevel_MT);

	//variables et structures utiles
	std::vector<unsigned> trianglesToTest;
	unsigned trianglesToTestCount=0;
	unsigned trianglesToTestCapacity=0;

	//Bit mask for efficient comparisons
	QBitArray* bitArray=0;
	if (s_useBitArrays_MT)
	{
		s_currentBitMaskMutex.lock();
		if (s_bitArrayPool_MT.empty())
		{
			bitArray = new QBitArray();
			bitArray->resize(s_theIntersection_MT->theMesh->size());
			//s_bitArrayPool_MT.push_back(bitArray);
		}
		else
		{
			bitArray = s_bitArrayPool_MT.back();
			s_bitArrayPool_MT.pop_back();
		}
		s_currentBitMaskMutex.unlock();
		bitArray->fill(0);
	}

	//on calcule pour chaque point sa distance au bord de la cellule la plus proche
	//cela nous permettra de recalculer plus rapidement la distance d'elligibilité
	//du triangle le plus proche
	Yk.placeIteratorAtBegining();
	for (unsigned j=0;j<remainingPoints;++j)
	{
		//coordonnées du point courant
		const CCVector3 *tempPt = Yk.getCurrentPointCoordinates();
		//distance du bord le plus proche = taille de la cellule - distance la plus grande par rapport au centre de la cellule
		//minDists.push_back(cellLength*0.5-DgmOctree::computeMaxDistanceToCellCenter(tempPt,cellCenter));
		minDists[j] = DgmOctree::ComputeMinDistanceToCellBorder(tempPt,cellLength,cellCenter);
		Yk.forwardIterator();
	}

	//initialisation de la récurrence
	DistanceType maxRadius=0;
	int dist=0;

	//on va essayer de trouver les triangles les plus proches de chaque point du "voisinage" Yk
	while (remainingPoints>0 && dist<=maxDist)
	{
		//on va tester les cellules voisines à une distance "dist"
		//a,b,c,d,e,f représentent l'extension spatiale du voisinage INCLUS DANS LA GRILLE 3D
		//selon les 6 directions -X,+X,-Y,+Y,-Z,+Z
		int a = ccMin(dist,distToLowerBorder[0]);
		int b = ccMin(dist,distToUpperBorder[0]);
		int c = ccMin(dist,distToLowerBorder[1]);
		int d = ccMin(dist,distToUpperBorder[1]);
		int e = ccMin(dist,distToLowerBorder[2]);
		int f = ccMin(dist,distToUpperBorder[2]);

		int index0 = startPos[0]-a;
		for (int i=-a;i<=b;i++)
		{
			bool imax = (abs(i)==dist);
			facesInCellPtr *_tab0 = s_theIntersection_MT->tab[index0];

			//index0 = (startPos[0]=i)*dec2;
			int index = (startPos[1]-c)*(int)s_theIntersection_MT->dec;
			for (int j=-c;j<=d;j++)
			{
				//index = index0+(startPos[1]+j)*dec1;

				//si i ou j est maximal
				if (imax || abs(j)==dist)
				{
					//on est forcément sur le bord du voisinage
					facesInCellPtr *_tab = _tab0+(index+startPos[2]-e);

					for (int k=-e;k<=f;k++)
					{
						//element correspondant
						if (*_tab)
						{
							const facesInCellPtr& element = *_tab;
							if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceCount;++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas déja été inséré
									if (!bitArray->testBit(indexTri))
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										bitArray->setBit(indexTri);
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
								}
							}
						}

						++_tab;
					}
				}
				else //on doit se mettre au bord du cube
				{
					if (e==dist) //côté négatif
					{
						//index correspondant
						const facesInCellPtr& element = _tab0[index+startPos[2]-e];

						if (element)
						{
							if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceCount;++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas déja été inséré
									if (!bitArray->testBit(indexTri))
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										bitArray->setBit(indexTri);
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
								}
							}
						}
					}

					if (f==dist && dist>0) //côté positif
					{
						//index correspondant
						const facesInCellPtr& element = _tab0[index+startPos[2]+f];

						if (element)
						{
							if (trianglesToTestCount + element->faceCount > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceCount,2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceCount;++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas déja été inséré
									if (!bitArray->testBit(indexTri))
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										bitArray->setBit(indexTri);
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
								}
							}
						}
					}
				} // fin else

				index += (int)s_theIntersection_MT->dec;
			}

			index0++;
		}

		bool firstComparisonDone = (trianglesToTestCount!=0);

		//pour chaque triangle
		while (trianglesToTestCount != 0)
		{
			//on récupère les coordonnées de ses sommets (references)
			CCLib::SimpleTriangle tri;
			s_theIntersection_MT->theMesh->getTriangleSummits(trianglesToTest[--trianglesToTestCount],tri.A,tri.B,tri.C);

			//pour chaque point dans la cellule
			Yk.placeIteratorAtBegining();
			if (s_signedDistances_MT)
			{
				for (unsigned j=0;j<remainingPoints;++j)
				{
					//on calcule la distance point/triangle (attention, on travaille avec des distances au carré)
					DistanceType dPTri = /*s_normalSign_MT **/ DistanceComputationTools::computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),&tri,true);
					//si elle est plus petite que la distance actuelle du point, on remplace
					DistanceType min_d = Yk.getCurrentPointScalarValue();
					if (min_d*min_d > dPTri*dPTri)
						Yk.setCurrentPointScalarValue(s_normalSign_MT*dPTri);
					Yk.forwardIterator();
				}
			}
			else
			{
				for (unsigned j=0;j<remainingPoints;++j)
				{
					//on calcule la distance point/triangle (attention, on travaille avec des distances au carré)
					DistanceType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),&tri,false);
					//si elle est plus petite que la distance actuelle du point, on remplace
					DistanceType min_d = Yk.getCurrentPointScalarValue();
					if (min_d < 0 || dPTri<min_d)
						Yk.setCurrentPointScalarValue(dPTri);
					Yk.forwardIterator();
				}
			}
		}

		//maintenant on retire tous les points "élus" lors de ce tour
		if (firstComparisonDone)
		{
			Yk.placeIteratorAtBegining();
			for (unsigned j=0;j<remainingPoints;++j)
			{
				//distance d'elligibilité
				DistanceType elligibleDist = minDists[j]+maxRadius;
				DistanceType dPTri = Yk.getCurrentPointScalarValue();
				if (s_signedDistances_MT)
					dPTri*=dPTri;
				if (dPTri <= elligibleDist*elligibleDist)
				{
					//on supprime le point courant
					Yk.removeCurrentPointGlobalIndex();
					//on applique l'opération équivalente (voir ReferenceCloud::removeCurrentPointGlobalIndex)
					//au tableau donnant la distance minimale par rapport à la cellule
					assert(remainingPoints>0);
					minDists[j] = minDists[remainingPoints-1];
					//minDists.pop_back();
					--remainingPoints;
					--j;

				}
				else Yk.forwardIterator();
			}
		}
		//*/

		++dist;
		maxRadius += cellLength;
	}

	if (minDists)
		delete[] minDists;
	minDists=0;

	//release bit mask
	if (bitArray)
	{
		s_currentBitMaskMutex.lock();
		s_bitArrayPool_MT.push_back(bitArray);
		s_currentBitMaskMutex.unlock();
	}
}

//Calcul de la distance signée entre un maillage et un nuage, accéléré par la structure "OctreeAndMeshIntersection" (multi-threaded)
int DistanceComputationTools::computePointCloud2MeshDistanceWithOctree_MT(OctreeAndMeshIntersection* theIntersection,
																		  uchar octreeLevel,
																		  bool signedDistances,
																		  bool flipTriangleNormals/*=false*/,
																		  GenericProgressCallback* progressCb/*=0*/)
{
	assert(theIntersection);
	assert(!theIntersection->distanceTransform);
	DgmOctree* theOctree = theIntersection->theOctree;

	//extraction des indexes et codes des cellules du niveau "octreeLevel"
	DgmOctree::cellsContainer cellsDescs;
	theOctree->getCellCodesAndIndexes(octreeLevel,cellsDescs,true);

	unsigned numberOfCells = cellsDescs.size();

	//Progress callback
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfCells);
		char buffer[256];
		sprintf(buffer,"Cells=%i",numberOfCells);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle("Compute signed distances");
		progressCb->start();
	}

	s_octree_MT = theOctree;
	s_normProgressCb_MT = nProgress;
	s_cellFunc_MT_success = true;
	s_signedDistances_MT = signedDistances;
	s_normalSign_MT = (flipTriangleNormals ? -1.0f : 1.0f);
	s_octreeLevel_MT = octreeLevel;
	s_theIntersection_MT = theIntersection;
	//acceleration structure
	s_useBitArrays_MT = true;

	//Single thread emulation
	//for (unsigned i=0; i<numberOfCells; ++i)
	//	cloudMeshDistCellFunc_MT(cellsDescs[i]);

	QtConcurrent::blockingMap(cellsDescs, cloudMeshDistCellFunc_MT);

	s_octree_MT = 0;
	s_normProgressCb_MT = 0;
	s_theIntersection_MT = 0;

	//clean acceleration structure
	while (!s_bitArrayPool_MT.empty())
	{
		delete s_bitArrayPool_MT.back();
		s_bitArrayPool_MT.pop_back();
	}

	if (nProgress)
        delete nProgress;
	nProgress=0;

	return (s_cellFunc_MT_success ? 0 : -2);
}

#endif

//passage des distance d'un nuage à la racine carée
void applySqrtToPointDist(const CCVector3 &aPoint, DistanceType& aScalarValue)
{
	if (aScalarValue>0.0)
        aScalarValue=sqrt(aScalarValue);
}

//approche équivalente à celle de METRO (P. Cignoni, C. Rocchini and R. Scopigno)
int DistanceComputationTools::computePointCloud2MeshDistance(GenericIndexedCloudPersist* pointCloud,
                                                                GenericIndexedMesh* theMesh,
                                                                uchar octreeLevel,
                                                                DistanceType maxSearchDist,
                                                                bool useDistanceMap/*=false*/,
                                                                bool signedDistances/*=false*/,
                                                                bool flipNormals/*=false*/,
																bool multiThread/*=true*/,
                                                                GenericProgressCallback* progressCb/*=0*/,
                                                                DgmOctree* cloudOctree/*=0*/)
{
    assert(pointCloud && theMesh);

	//tests préliminaires sur les variables d'entrée
	if (pointCloud->size()==0 || theMesh->size()==0)
        return -2;

    //signed distances are incompatible with Chamfer approximation
    if (signedDistances)
        useDistanceMap = false;

	//on regarde si la boite englobante du nuage et du maillage coincïdent
	CCVector3 cloudMinBB,cloudMaxBB,meshMinBB,meshMaxBB,minBB,maxBB,minCubifiedBB,maxCubifiedBB;
	pointCloud->getBoundingBox(cloudMinBB.u,cloudMaxBB.u);
	theMesh->getBoundingBox(meshMinBB.u,meshMaxBB.u);

	//on calcule les limites de la boite englobante maximale
	int k;
	for (k=0;k<3;++k)
	{
		minBB.u[k] = ccMin(meshMinBB.u[k],cloudMinBB.u[k]);
		maxBB.u[k] = ccMax(meshMaxBB.u[k],cloudMaxBB.u[k]);
	}

	//on calcule les dimensions de l'octree correspondant
	minCubifiedBB = minBB;
	maxCubifiedBB = maxBB;
	CCMiscTools::makeMinAndMaxCubical(minCubifiedBB,maxCubifiedBB);

	//calcul de la structure octree
	DgmOctree* theOctree = cloudOctree;
	bool wrongOctreeDimensions=false;
	if (!theOctree)
	{
		theOctree = new DgmOctree(pointCloud);
		wrongOctreeDimensions=true;
	}
	else
	{
		//on vérifie que l'octree fourni en entrée a les bonnes dimensions
		const CCVector3& theOctreeMins = theOctree->getOctreeMins();
		const CCVector3& theOctreeMaxs = theOctree->getOctreeMaxs();
		for (k=0;k<3;++k)
		{
			if ( theOctreeMins.u[k] != minCubifiedBB.u[k] ||
                 theOctreeMaxs.u[k] != maxCubifiedBB.u[k])
			{
				wrongOctreeDimensions=true;
				break;
			}
		}
	}

	//on doit re(calculer) l'octree
	if (wrongOctreeDimensions)
	{
		if (theOctree->build(minCubifiedBB,maxCubifiedBB,&cloudMinBB,&cloudMaxBB,progressCb)<1)
		{
			if (!cloudOctree)
                delete theOctree;
			return -36;
		}
	}

	OctreeAndMeshIntersection theIntersection;

	//we deduce grid cell size very simply (as bbox has been "cubified")
	PointCoordinateType cellSize = (maxCubifiedBB.x-minCubifiedBB.x)/PointCoordinateType(1<<octreeLevel);
	//we compute grid occupancy ...
	//... and we deduce array dimensions
	unsigned tabSizes[3];
	for (k=0;k<3;++k)
	{
        theIntersection.minFillIndexes[k] = int(floor((minBB.u[k]-minCubifiedBB.u[k])/cellSize));
		theIntersection.maxFillIndexes[k] = int(floor((maxBB.u[k]-minCubifiedBB.u[k])/cellSize));
		tabSizes[k]=unsigned(theIntersection.maxFillIndexes[k]-theIntersection.minFillIndexes[k]+1);
	}

	theIntersection.theOctree = theOctree;
	theIntersection.theMesh = theMesh;
	theIntersection.dec = tabSizes[2];
	theIntersection.sliceNumber = tabSizes[0];
	theIntersection.sliceSize = tabSizes[1]*tabSizes[2];

	bool boundedSearch = (maxSearchDist>=0);
	multiThread &= (!boundedSearch); //MT doesn't support boundedSearch yet!
	if (!useDistanceMap || boundedSearch)
	{
		//structure contenant pour chaque cellule de la grille 3D
		//la liste des triangles intersectants
		//Rq : tableau à 2 "dimensions" pour éviter les blocs de mémoire trop gros
		theIntersection.tab = new facesInCellPtr*[theIntersection.sliceNumber];
		if (!theIntersection.tab)
		{
			//printf("Not enough memory !\n Try with a lower octree level ...");
			if (!cloudOctree)
                delete theIntersection.theOctree;
			return -4;
		}
		memset(theIntersection.tab,0,theIntersection.sliceNumber*sizeof(facesInCellPtr*));

		//plutôt que d'instancier d'un bloc la grille 3D, on instancie chaque tranche séparemment
		//comme ça on a plus de chance de trouver des segments de mémoire contigue suffisamment grands
		for (unsigned i=0;i<theIntersection.sliceNumber;++i)
		{
			theIntersection.tab[i] = new facesInCellPtr[theIntersection.sliceSize];
			if (!theIntersection.tab[i])
			{
				//printf("Not enough contiguous memory !\n Try with a lower octree level ...");
				if (!cloudOctree)
                    delete theIntersection.theOctree;
				return -4;
			}
			memset(theIntersection.tab[i],0,theIntersection.sliceSize*sizeof(facesInCellPtr*));
		}
	}
	else
	{
		//we only compute approximate (Chamfer) distances
		multiThread = false; //not necessary/supported
	}

	//si l'utilisateur (ou la configuration nuage/maillage) nécessite l'utilisation d'une transformée
	//de distance de type Chanfrein.
	if (useDistanceMap)
	{
		theIntersection.distanceTransform = new ChamferDistanceTransform(tabSizes[0],tabSizes[1],tabSizes[2]);
		if ( !theIntersection.distanceTransform || !theIntersection.distanceTransform->init())
		{
			//printf("Not enough memory for DT acceleration !\n Try with a lower octree level ...");
			if (!cloudOctree)
                delete theIntersection.theOctree;
			return -5;
		}
	}

	//ON INTERSECTE L'OCTREE AVEC LE MAILLAGE
	int result = intersectMeshWithOctree(&theIntersection,octreeLevel,progressCb);
	if (result < 0)
	{
		//printf("Something went wrong !\n");
		if (!cloudOctree)
            delete theIntersection.theOctree;
		return -6;
	}

	//raz des distances
	pointCloud->enableScalarField();
	if (signedDistances)
        pointCloud->forEach(ScalarFieldTools::razDistsToBigValue);
	else
        pointCloud->forEach(ScalarFieldTools::razDistsToHiddenValue);

    //acceleration par caclul d'une transformée de distance
	if (useDistanceMap && theIntersection.distanceTransform)
        theIntersection.distanceTransform->propagateDistance(CHAMFER_345, progressCb);

	//EVENTUALLY, WE CAN COMPUTE DISTANCES!

#ifdef ENABLE_CLOUD2MESH_DIST_MT
	//DGM: MT mode still under test!
	if (multiThread)
	{
		result = computePointCloud2MeshDistanceWithOctree_MT(&theIntersection,octreeLevel,signedDistances,flipNormals,progressCb);
	}
	else
#endif
	{
		result = computePointCloud2MeshDistanceWithOctree(&theIntersection,octreeLevel,signedDistances,flipNormals,maxSearchDist,progressCb);
	}

	//special operation for non-signed distances
	if (result ==0 && !signedDistances)
	{
		//don't forget to pass the result to the square root
		if (!useDistanceMap || boundedSearch)
			pointCloud->forEach(applySqrtToPointDist);
	}


	if (!cloudOctree)
        delete theIntersection.theOctree;

	if (result < 0)
        return -7;

	return 0;
}

/******* Calcul de distance entre un point et un triangle *****/
// Inspired from Magic Software, Inc.
// http://www.magic-software.com
// http://www.wild-magic.com
// Copyright (c) 2004.  All Rights Reserved
//
// The Wild Magic Library (WML) source code is supplied under the terms of
// the license agreement http://www.magic-software.com/License/WildMagic.pdf
// and may not be copied or disclosed except in accordance with the terms of
// that agreement.
DistanceType DistanceComputationTools::computePoint2TriangleDistance(const CCVector3* P, const GenericTriangle* theTriangle, bool signedDist)
{
    assert(P && theTriangle);

	const CCVector3* A = theTriangle->_getA();
	const CCVector3* B = theTriangle->_getB();
	const CCVector3* C = theTriangle->_getC();

	//we do all computations with double precision, otherwise
	//some triangles with sharp angles will give very poor results.
	double AP[3]={P->x-A->x,P->y-A->y,P->z-A->z};
	double AB[3]={B->x-A->x,B->y-A->y,B->z-A->z};
	double AC[3]={C->x-A->x,C->y-A->y,C->z-A->z};

	double fSqrDist;
	{
		double fA00 = CCVector3::vdot(AB,AB);
		double fA11 = CCVector3::vdot(AC,AC);
		double fA01 = CCVector3::vdot(AB,AC);
		double fB0 = -CCVector3::vdot(AP,AB);
		double fB1 = -CCVector3::vdot(AP,AC);
		fSqrDist = /*double fC =*/ CCVector3::vdot(AP,AP); //DGM: in fact, fSqrDist is always equal to fC + something
		double fDet = fabs(fA00*fA11-fA01*fA01);
		double fS = fA01*fB1-fA11*fB0;
		double fT = fA01*fB0-fA00*fB1;

		if ( fS + fT <= fDet )
		{
			if ( fS < 0 )
			{
				if ( fT < 0 )  // region 4
				{
					if ( fB0 < 0 )
					{
						if ( -fB0 >= fA00 )
						{
							fSqrDist += fA00+2.0*fB0/*+fC*/;
						}
						else
						{
							fSqrDist += -fB0*fB0/fA00/*+fC*/;
						}
					}
					else
					{
						if ( fB1 >= 0 )
						{
							//fSqrDist = fC;
						}
						else if ( -fB1 >= fA11 )
						{
							fSqrDist += fA11+2.0*fB1/*+fC*/;
						}
						else
						{
							fSqrDist += -fB1*fB1/fA11/*+fC*/;
						}
					}
				}
				else  // region 3
				{
					if ( fB1 >= 0 )
					{
						//fSqrDist += fC;
					}
					else if ( -fB1 >= fA11 )
					{
						fSqrDist += fA11+2.0*fB1/*+fC*/;
					}
					else
					{
						fSqrDist += -fB1*fB1/fA11/*+fC*/;
					}
				}
			}
			else if ( fT < 0 )  // region 5
			{
				if ( fB0 >= 0 )
				{
					//fSqrDist += fC;
				}
				else if ( -fB0 >= fA00 )
				{
					fSqrDist += fA00+2.0*fB0/*+fC*/;
				}
				else
				{
					fSqrDist += -fB0*fB0/fA00/*+fC*/;
				}
			}
			else  // region 0
			{
				// minimum at interior point
				fS /= fDet;
				fT /= fDet;
				fSqrDist += fS*(fA00*fS+fA01*fT+2.0*fB0) + fT*(fA01*fS+fA11*fT+2.0*fB1) /*+fC*/;
			}
		}
		else
		{
			if ( fS < 0 )  // region 2
			{
				double fTmp0 = fA01 + fB0;
				double fTmp1 = fA11 + fB1;
				if ( fTmp1 > fTmp0 )
				{
					double fNumer = fTmp1 - fTmp0;
					double fDenom = fA00-2.0*fA01+fA11;
					if ( fNumer >= fDenom )
					{
						fSqrDist += fA00+2.0*fB0/*+fC*/;
					}
					else
					{
						fS = fNumer/fDenom;
						fT = 1.0 - fS;
						fSqrDist += fS*(fA00*fS+fA01*fT+2.0*fB0) + fT*(fA01*fS+fA11*fT+2.0*fB1)/*+fC*/;
					}
				}
				else
				{
					if ( fTmp1 <= 0 )
					{
						fSqrDist += fA11+2.0*fB1/*+fC*/;
					}
					else /*if ( fB1 >= 0 )
					{
						fSqrDist += fC;
					}
					else*/ if (fB1 < 0)
					{
						fSqrDist += -fB1*fB1/fA11/*+fC*/;
					}
				}
			}
			else if ( fT < 0 )  // region 6
			{
				double fTmp0 = fA01 + fB1;
				double fTmp1 = fA00 + fB0;
				if ( fTmp1 > fTmp0 )
				{
					double fNumer = fTmp1 - fTmp0;
					double fDenom = fA00-2.0*fA01+fA11;
					if ( fNumer >= fDenom )
					{
						fSqrDist += fA11+(2.0)*fB1/*+fC*/;
					}
					else
					{
						fT = fNumer/fDenom;
						fS = 1.0 - fT;
						fSqrDist += fS*(fA00*fS+fA01*fT+2.0*fB0) + fT*(fA01*fS+fA11*fT+2.0*fB1)/*+fC*/;
					}
				}
				else
				{
					if ( fTmp1 <= 0 )
					{
						fSqrDist += fA00+2.0*fB0/*+fC*/;
					}
					else /*if ( fB0 >= 0 )
					{
						fSqrDist += fC;
					}
					else*/ if ( fB0 < 0 )
					{
						fSqrDist += -fB0*fB0/fA00/*+fC*/;
					}
				}
			}
			else  // region 1
			{
				double fNumer = fA11 + fB1 - fA01 - fB0;
				if ( fNumer <= 0 )
				{
					fSqrDist += fA11+2.0*fB1/*+fC*/;
				}
				else
				{
					double fDenom = fA00-2.0*fA01+fA11;
					if ( fNumer >= fDenom )
					{
						fSqrDist += fA00+2.0*fB0/*+fC*/;
					}
					else
					{
						fS = fNumer/fDenom;
						fT = 1.0 - fS;
						fSqrDist += fS*(fA00*fS+fA01*fT+2.0*fB0) + fT*(fA01*fS+fA11*fT+2.0*fB1)/*+fC*/;
					}
				}
			}
		}
	}

	if (signedDist)
	{
		DistanceType d = (DistanceType)sqrt(fabs(fSqrDist)); //fabs --> il arrive que le calcul ne soit pas hyper précis !

		//triangle normal
		double N[3];
		CCVector3::vcross(AB,AC,N);

		//we test the sign of the cross product of the triangle normal and the vector AP
		return (CCVector3::vdot(AP,N)<0 ? -d : d);
	}
	else
	{
		return (DistanceType)fabs(fSqrDist);  //fabs --> il arrive que le calcul ne soit pas hyper précis !
	}
}

//calcule la distance de P(xp,yp,zp) par rapport au plan : a0.x + a1.y + a2.z = a3
DistanceType DistanceComputationTools::computePoint2PlaneDistance(const CCVector3* P, const PointCoordinateType* aPlane)
{
	//distance d'un point à un plan : d = fabs(a0*x+a1*y+a2*z-a3)/sqrt(a0²+a1²+a2²)
	DistanceType norm2 = CCVector3::vnorm2(aPlane);

	if (norm2 < ZERO_TOLERANCE)
        return HIDDEN_VALUE;

	return fabs((DistanceType)(CCVector3::vdot(P->u,aPlane)-aPlane[3]))/sqrt(norm2);
}

//calcule la distance d'un nuage de point par rapport au plan : a0.x + a1.y + a2.z = a3
DistanceType DistanceComputationTools::computeCloud2PlaneDistance(GenericCloud* theCloud, const PointCoordinateType* thePlaneEquation)
{
    assert(theCloud && thePlaneEquation);

	//nombre de points
	unsigned n = theCloud->size();
	//distance d'un point à un plan : d = fabs(a0*x+a1*y+a2*z-a3) / sqrt(a0²+a1²+a2²) <-- "norm"
	double norm = CCVector3::vnorm(thePlaneEquation);

	if (norm==0.0 || n==0)
        return HIDDEN_VALUE;

	double dSum2 = 0.0;

	//calcul des différences
	theCloud->placeIteratorAtBegining();
	for (unsigned i=0;i<n;++i)
	{
		const CCVector3* P = theCloud->getNextPoint();
		dSum2 += fabs((double)(CCVector3::vdot(P->u,thePlaneEquation)-thePlaneEquation[3]));
	}

	return DistanceType(dSum2/(norm*double(n)));
}


bool DistanceComputationTools::computeGeodesicDistances(GenericIndexedCloudPersist* theCloud, unsigned seedPointIndex, uchar octreeLevel, GenericProgressCallback* progressCb)
{
    assert(theCloud);

	unsigned n=theCloud->size();
	if (n<1 || seedPointIndex>=n)
        return false;

	theCloud->enableScalarField();
	theCloud->forEach(ScalarFieldTools::razDistsToHiddenValue);

	DgmOctree* theOctree = new DgmOctree(theCloud);
	if (theOctree->build(progressCb)<1)
	{
		delete theOctree;
		return false;
	}

	FastMarchingForPropagation* fm = new FastMarchingForPropagation();

	if (fm->init(theCloud,theOctree,octreeLevel,true)<0)
	{
		//fl_alert("Probleme lors de l'initialisation");
		delete theOctree;
		delete fm;
		return false;
	}

	//on cherche la cellule de l'octree qui englobe le "seedPoint"
	int cellPos[3];
	theOctree->getTheCellPosWhichIncludesThePoint(theCloud->getPoint(seedPointIndex),cellPos,octreeLevel);
	fm->setSeedCell(cellPos);
	bool result=false;
	if (fm->propagate()>=0)
		result=fm->setPropagationTimingsAsDistances();

	delete fm;
	delete theOctree;

	return result;
}

int DistanceComputationTools::diff(GenericIndexedCloudPersist* comparedCloud, GenericIndexedCloudPersist* referenceCloud, bool comparedSFisPositive, bool referenceSFisPositive, GenericProgressCallback* progressCb)
{
	if (!comparedCloud || !referenceCloud)
		return -1;

	unsigned nA=comparedCloud->size();
	if (nA<1)
		return -2;

	//Reference cloud to store closest point set
	ReferenceCloud A_in_B(referenceCloud);

	Cloud2CloudDistanceComputationParams params;
	params.octreeLevel = DgmOctree::MAX_OCTREE_LEVEL-1;
	params.CPSet = &A_in_B;

	int result = computeHausdorffDistance(comparedCloud,referenceCloud,params,progressCb);
	if (result<0)
		return -3;

	DistanceType dA,dB;
	unsigned i;
	for (i=0;i<nA;++i)
	{
		//on admet que les points sont bien homologues !
		dA = comparedCloud->getPointScalarValue(i);
		dB = A_in_B.getPointScalarValue(i);
		//Valeurs non valides :
		//- "< 0.0" si champ scalaire "positif"
		if (((comparedSFisPositive && (dA<0.0))||(referenceSFisPositive && (dB<0.0)))
			//- "> BIG_VALUE" si champ scalaire "quelconque"
			||((!comparedSFisPositive && (dA>=BIG_VALUE))||(!referenceSFisPositive && (dB>=BIG_VALUE))))
			comparedCloud->setPointScalarValue(i,BIG_VALUE);
		else
			comparedCloud->setPointScalarValue(i,dA-dB);
	}

	return 0;
}

int DistanceComputationTools::computeChamferDistanceBetweenTwoClouds(CC_CHAMFER_DISTANCE_TYPE cType, GenericIndexedCloudPersist* comparedCloud, GenericIndexedCloudPersist* referenceCloud, uchar octreeLevel, GenericProgressCallback* progressCb, DgmOctree* compOctree, DgmOctree* refOctree)
{
	if (!comparedCloud || !referenceCloud)
		return -1;
	if (octreeLevel<1 || octreeLevel>DgmOctree::MAX_OCTREE_LEVEL)
		return -2;

	//on calcul les octree synchro (même bounding-box)
	DgmOctree *octreeA=compOctree,*octreeB=refOctree;
	if (!synchronizeOctrees(comparedCloud,referenceCloud,octreeA,octreeB,progressCb))
		return -3;

	const int* minIndexesA = octreeA->getMinFillIndexes(octreeLevel);
	const int* maxIndexesA = octreeA->getMaxFillIndexes(octreeLevel);
	const int* minIndexesB = octreeA->getMinFillIndexes(octreeLevel);
	const int* maxIndexesB = octreeA->getMaxFillIndexes(octreeLevel);

	int minIndexes[3],maxIndexes[3];
	minIndexes[0]=ccMin(minIndexesA[0],minIndexesB[0]);
	minIndexes[1]=ccMin(minIndexesA[1],minIndexesB[1]);
	minIndexes[2]=ccMin(minIndexesA[2],minIndexesB[2]);
	maxIndexes[0]=ccMax(maxIndexesA[0],maxIndexesB[0]);
	maxIndexes[1]=ccMax(maxIndexesA[1],maxIndexesB[1]);
	maxIndexes[2]=ccMax(maxIndexesA[2],maxIndexesB[2]);

	unsigned short boxSize[3];
	boxSize[0] = (unsigned short)(maxIndexes[0] - minIndexes[0]+1);
	boxSize[1] = (unsigned short)(maxIndexes[1] - minIndexes[1]+1);
	boxSize[2] = (unsigned short)(maxIndexes[2] - minIndexes[2]+1);

	//Console::print("BoxSize = (%i,%i,%i)\n",boxSize[0],boxSize[1],boxSize[2]);

	//puis on projette les cellules de l'octree B dans un grille 3D (DistanceGrid)
	ChamferDistanceTransform* dg = new ChamferDistanceTransform(boxSize[0],boxSize[1],boxSize[2]);
	if (!dg->init())
	{
		delete dg;
		if (!compOctree) delete octreeA;
		if (!refOctree) delete octreeB;
		return -4;
	}

	DgmOctree::cellCodesContainer theCodes;
	octreeB->getCellCodes(octreeLevel,theCodes,true);
	OctreeCellCodeType theCode;

	int pos[3];
	while (!theCodes.empty())
	{
		theCode = theCodes.back();
		theCodes.pop_back();
		octreeB->getCellPos(theCode,octreeLevel,pos,true);
		pos[0] -= minIndexes[0];
		pos[1] -= minIndexes[1];
		pos[2] -= minIndexes[2];
		dg->setZero(pos);
	}

	//propagation de la transformée de distance
	dg->propagateDistance(cType,progressCb);

	//maintenant on récupère les cellules de l'octree A, et on assigne
	//aux points la distance de Chanfrein
	PointCoordinateType cellSize = octreeA->getCellSize(octreeLevel);
	if (cType==CHAMFER_345)
		cellSize*=0.3333f;

	DgmOctree::cellIndexesContainer theIndexes;
	octreeA->getCellIndexes(octreeLevel,theIndexes);
	unsigned theIndex;

	comparedCloud->enableScalarField();

	unsigned maxDi=0;
	ReferenceCloud Yk(octreeA->associatedCloud());

	while (!theIndexes.empty())
	{
		theIndex = theIndexes.back();
		theIndexes.pop_back();

		octreeA->getCellPos(octreeA->getCellCode(theIndex),octreeLevel,pos,false);
		pos[0] -= minIndexes[0];
		pos[1] -= minIndexes[1];
		pos[2] -= minIndexes[2];
		unsigned di = dg->getValue(pos);
		if (di>maxDi)
			maxDi = di;
		DistanceType d = DistanceType(di)*cellSize;

		octreeA->getPointsInCellByCellIndex(&Yk,theIndex,octreeLevel);
		for (unsigned j=0;j<Yk.size();++j)
			Yk.setPointScalarValue(j,d);
	}

	delete dg;
	dg=0;

	if (!compOctree)
	{
		delete octreeA;
		octreeA=0;
	}
	if (!refOctree)
	{
		delete octreeB;
		octreeB=0;
	}

	if (cType==CHAMFER_345)
		maxDi/=3;

	return maxDi;
}
