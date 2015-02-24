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

#include "DistanceComputationTools.h"

//local
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
#include "CCConst.h"
#include "CCMiscTools.h"
#include "LocalModel.h"
#include "SimpleTriangle.h"
#include "ScalarField.h"

//system
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <limits>

using namespace CCLib;

namespace CCLib
{

	//! Internal structure used by DistanceComputationTools::computePointCloud2MeshDistance
	struct FacesInCell
	{
	public:
		//! cell code
		DgmOctree::OctreeCellCodeType cellCode;
		//! Indexes of all faces inside cell
		std::vector<unsigned> faceIndexes;

		//! Default constructor
		FacesInCell()
			: cellCode(0)
		{
		}

		//! Add a face index to 'faceIndexes'
		inline bool push(unsigned faceIndex)
		{
			try
			{
				faceIndexes.push_back(faceIndex);
			}
			catch(std::bad_alloc)
			{
				return false;
			}
			return true;
		}
	};

	//! Pointer on a FacesInCell structure
	typedef FacesInCell* FacesInCellPtr;

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

		//! Grid dimension
		static const unsigned GRID_DIMENSION = 3;

		//! Grid occupancy of mesh (minimum indexes for each dimension)
		int minFillIndexes[GRID_DIMENSION];
		//! Grid occupancy of mesh (maximum indexes for each dimension)
		int maxFillIndexes[GRID_DIMENSION];

		//! Array of FacesInCellPtr structures
		FacesInCellPtr **tab;
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
			minFillIndexes[0] = minFillIndexes[1] = minFillIndexes[2] = 0;
			maxFillIndexes[0] = maxFillIndexes[1] = maxFillIndexes[2] = 0;
		}

		//! Destructor
		~OctreeAndMeshIntersection()
		{
			if (tab)
			{
				//we clear the intersection structure
				for (unsigned i=0;i<sliceNumber;++i)
				{
					if (tab[i])
					{
						for (unsigned j=0;j<sliceSize;++j)
							if (tab[i][j])
								delete tab[i][j];
						delete[] tab[i];
					}
				}

				delete[] tab;
				tab=0;
			}

			if (distanceTransform)
			{
				delete distanceTransform;
				distanceTransform = 0;
			}
		}
	};
}

int DistanceComputationTools::computeHausdorffDistance(	GenericIndexedCloudPersist* comparedCloud,
														GenericIndexedCloudPersist* referenceCloud,
														Cloud2CloudDistanceComputationParams& params,
														GenericProgressCallback* progressCb/*=0*/,
														DgmOctree* compOctree/*=0*/,
														DgmOctree* refOctree/*=0*/)
{
	assert(comparedCloud && referenceCloud);

	if (params.CPSet && params.maxSearchDist >= 0)
	{
		//we can't use a 'max search distance' criterion if the "Closest Point Set" is requested
		return -666;
	}

	//we spatially 'synchronize' the octrees
	DgmOctree *comparedOctree = compOctree, *referenceOctree = refOctree;
	SOReturnCode soCode = synchronizeOctrees(	comparedCloud,
												referenceCloud,
												comparedOctree,
												referenceOctree,
												params.maxSearchDist,
												progressCb);
	
	if (soCode != SYNCHRONIZED && soCode != DISJOINT)
	{
		//not enough memory (or invalid input)
		return -1;
	}

	//we 'enable' a scalar field  (if it is not already done) to store resulting distances
	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return -1;
	}

	//internally we don't use the maxSearchDist parameters as is, but the square of it
	double maxSearchSquareDistd = params.maxSearchDist < 0 ? -1.0 : static_cast<double>(params.maxSearchDist)*params.maxSearchDist;

	//closest point set
	if (params.CPSet)
	{
		assert(maxSearchSquareDistd < 0);
		
		if (!params.CPSet->resize(comparedCloud->size()))
		{
			//not enough memory
			if (comparedOctree && !compOctree)
				delete comparedOctree;
			if (referenceOctree && !refOctree)
				delete referenceOctree;
			return -1;
		}
	}

	//by default we reset any former value stored in the 'enabled' scalar field
	const ScalarType resetValue = maxSearchSquareDistd < 0 ? NAN_VALUE : params.maxSearchDist;
	if (params.resetFormerDistances)
	{
		for (unsigned i=0; i<comparedCloud->size(); ++i)
			comparedCloud->setPointScalarValue(i,resetValue);
	}

	//specific case: a max search distance has been defined and octrees are totally disjoint
	if (maxSearchSquareDistd >= 0 && soCode == DISJOINT)
	{
		//nothing to do! (all points are farther than 'maxSearchDist'
		return 0;
	}

	//if necessary we try to guess the best octree level for distances computation
	if (params.octreeLevel == 0)
	{
		params.octreeLevel = comparedOctree->findBestLevelForComparisonWithOctree(referenceOctree);
	}

	//additional parameters
	void* additionalParameters[4] = {	(void*)referenceCloud,
										(void*)referenceOctree,
										(void*)&params,
										(void*)&maxSearchSquareDistd
	};

	int result = 0;

	bool success = false;
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
		success = (comparedOctree->executeFunctionForAllCellsAtLevel(	params.octreeLevel,
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

	if (comparedOctree && !compOctree)
	{
		delete comparedOctree;
		comparedOctree = 0;
	}
	if (referenceOctree && !refOctree)
	{
		delete referenceOctree;
		referenceOctree = 0;
	}

	return result;
}

DistanceComputationTools::SOReturnCode
	DistanceComputationTools::synchronizeOctrees(	GenericIndexedCloudPersist* comparedCloud,
													GenericIndexedCloudPersist* referenceCloud,
													DgmOctree* &comparedOctree,
													DgmOctree* &referenceOctree,
													PointCoordinateType maxDist,
													GenericProgressCallback* progressCb/*=0*/)
{
	assert(comparedCloud && referenceCloud);

	unsigned nA = comparedCloud->size();
	unsigned nB = referenceCloud->size();

	if (nA == 0 || nB == 0)
		return EMPTY_CLOUD;

	//we compute the bounding box of BOTH clouds
	CCVector3 minsA,minsB,maxsA,maxsB;
	comparedCloud->getBoundingBox(minsA.u,maxsA.u);
	referenceCloud->getBoundingBox(minsB.u,maxsB.u);

	CCVector3 maxD,minD;
	//we compute the union of both bounding-boxes
	for (uchar k=0; k<3; k++)
	{
		minD.u[k] = std::min(minsA.u[k],minsB.u[k]);
		maxD.u[k] = std::max(maxsA.u[k],maxsB.u[k]);
	}

	if (maxDist >= 0)
	{
		//we reduce the bouding box to the intersection of both bounding-boxes enlarged by 'maxDist'
		for (uchar k=0; k<3; k++)
		{
			minD.u[k] = std::max(minD.u[k],std::max(minsA.u[k],minsB.u[k])-maxDist);
			maxD.u[k] = std::min(maxD.u[k],std::min(maxsA.u[k],maxsB.u[k])+maxDist);
			if (minD.u[k] > maxD.u[k])
			{
				return DISJOINT;
			}
		}
	}

	CCVector3 minPoints = minD;
	CCVector3 maxPoints = maxD;

	//we make this bounding-box cubical (+1% growth to avoid round-off issues)
	CCMiscTools::MakeMinAndMaxCubical(minD,maxD,0.01);

	//then we (re)compute octree A if necessary
	bool needToRecalculateOctreeA = true;
	if (comparedOctree && comparedOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeA = false;
		for (uchar k=0; k<3; k++)
		{
			if (	maxD.u[k] != comparedOctree->getOctreeMaxs().u[k]
				||	minD.u[k] != comparedOctree->getOctreeMins().u[k] )
			{
				needToRecalculateOctreeA = true;
				break;
			}
		}
	}

	bool octreeACreated = false;
	if (needToRecalculateOctreeA)
	{
		if (comparedOctree)
		{
			comparedOctree->clear();
		}
		else
		{
			comparedOctree = new DgmOctree(comparedCloud);
			octreeACreated = true;
		}

		if (comparedOctree->build(minD,maxD,&minPoints,&maxPoints,progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = 0;
			}
			return OUT_OF_MEMORY;
		}
	}

	//and we (re)compute octree B as well if necessary
	bool needToRecalculateOctreeB = true;
	if (referenceOctree && referenceOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeB = false;
		for (uchar k=0; k<3; k++)
		{
			if (	maxD.u[k] != referenceOctree->getOctreeMaxs().u[k]
				||	minD.u[k] != referenceOctree->getOctreeMins().u[k] )
			{
				needToRecalculateOctreeB = true;
				break;
			}
		}
	}

	if (needToRecalculateOctreeB)
	{
		bool octreeBCreated = false;
		if (referenceOctree)
		{
			referenceOctree->clear();
		}
		else
		{
			referenceOctree = new DgmOctree(referenceCloud);
			octreeBCreated = true;
		}

		if (referenceOctree->build(minD,maxD,&minPoints,&maxPoints,progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = 0;
			}
			if (octreeBCreated)
			{
				delete referenceOctree;
				referenceOctree = 0;
			}
			return OUT_OF_MEMORY;
		}
	}

	//we check that both octrees are ok
	assert(comparedOctree && referenceOctree);
	assert(comparedOctree->getNumberOfProjectedPoints() != 0 && referenceOctree->getNumberOfProjectedPoints() != 0);
	return SYNCHRONIZED;
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistanceComputationParams*): parameters
// [3] -> (ScalarType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistance(const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	const GenericIndexedCloudPersist* referenceCloud	= (GenericIndexedCloudPersist*)additionalParameters[0];
	const DgmOctree* referenceOctree					= (DgmOctree*)additionalParameters[1];
	Cloud2CloudDistanceComputationParams* params		= (Cloud2CloudDistanceComputationParams*)additionalParameters[2];
	const double* maxSearchSquareDistd					= (double*)additionalParameters[3];

	//structure for the nearest neighbor search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDistd				= *maxSearchSquareDistd;

	//we can already compute the position of the 'equivalent' cell in the reference octree
	referenceOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//for each point of the current cell (compared octree) we look for its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i=0; i<pointCount; i++)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		if (params->CPSet || referenceCloud->testVisibility(nNSS.queryPoint) == POINT_VISIBLE) //to build the closest point set up we must process the point whatever its visibility is!
		{
			double squareDist = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS);
			if (squareDist >= 0)
			{
				ScalarType dist = static_cast<ScalarType>(sqrt(squareDist));
				cell.points->setPointScalarValue(i,dist);

				if (params->CPSet)
					params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i),nNSS.theNearestPointIndex);
			}
			else
			{
				assert(!params->CPSet);
			}
		}
		else
		{
			cell.points->setPointScalarValue(i,NAN_VALUE);
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

//Description of expected 'additionalParameters'
// [0] -> (GenericIndexedCloudPersist*) reference cloud
// [1] -> (Octree*): reference cloud octree
// [2] -> (Cloud2CloudDistanceComputationParams*): parameters
// [3] -> (ScalarType*): max search distance (squared)
bool DistanceComputationTools::computeCellHausdorffDistanceWithLocalModel(	const DgmOctree::octreeCell& cell,
																			void** additionalParameters,
																			NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	GenericIndexedCloudPersist* referenceCloud		= (GenericIndexedCloudPersist*)additionalParameters[0];
	const DgmOctree* referenceOctree				= (DgmOctree*)additionalParameters[1];
	Cloud2CloudDistanceComputationParams* params	= (Cloud2CloudDistanceComputationParams*)additionalParameters[2];
	const double* maxSearchSquareDistd				= (double*)additionalParameters[3];

	assert(params && params->localModel != NO_MODEL);

	//structure for the nearest neighbor seach
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDistd				= *maxSearchSquareDistd;
	//we already compute the position of the 'equivalent' cell in the reference octree
	referenceOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//structures for determining the nearest neighbours of the 'nearest neighbour' (to compute the local model)
	//either inside a sphere or the k nearest
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS_Model;
	nNSS_Model.level = cell.level;
	if (params->useSphericalSearchForLocalModel)
	{
		nNSS_Model.prepare(static_cast<PointCoordinateType>(params->radiusForLocalModel),cell.parentOctree->getCellSize(cell.level));
		//curent cell (DGM: is it necessary? This is not always the right one)
		//memcpy(nNSS_Model_spherical.cellCenter,nNSS.cellCenter,3*sizeof(PointCoordinateType));
		//memcpy(nNSS_Model_spherical.cellPos,nNSS.cellPos,3*sizeof(int));
	}
	else
	{
		nNSS_Model.minNumberOfNeighbors = params->kNNForLocalModel;
		//curent cell (DGM: is it necessary? This is not always the right one)
		//memcpy(nNSS_Model_kNN.cellCenter,nNSS.cellCenter,3*sizeof(PointCoordinateType));
		//memcpy(nNSS_Model_kNN.cellPos,nNSS.cellPos,3*sizeof(int));
	}

	//already computed models
	std::vector<const LocalModel*> models;

	//for each point of the current cell (compared octree) we look its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i=0; i<pointCount; ++i)
	{
		//distance of the current point
		ScalarType distPt = NAN_VALUE;

		cell.points->getPoint(i,nNSS.queryPoint);
		if (params->CPSet || referenceCloud->testVisibility(nNSS.queryPoint) == POINT_VISIBLE) //to build the closest point set up we must process the point whatever its visibility is!
		{
			//first, we look for the nearest point to "_queryPoint" in the reference cloud
			double squareDistToNearestPoint = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS);

			//if it exists
			if (squareDistToNearestPoint >= 0)
			{
				ScalarType distToNearestPoint = static_cast<ScalarType>(sqrt(squareDistToNearestPoint));

				CCVector3 nearestPoint;
				referenceCloud->getPoint(nNSS.theNearestPointIndex,nearestPoint);

				//local model for the 'nearest point'
				const LocalModel* lm = 0;

				if (params->reuseExistingLocalModels)
				{
					//we look if the nearest point is close to existing models
					for (std::vector<const LocalModel*>::const_iterator it = models.begin(); it!=models.end(); ++it)
					{
						//we take the first model that 'includes' the nearest point
						if ( ((*it)->getCenter() - nearestPoint).norm2() <= (*it)->getSquareSize())
						{
							lm = *it;
							break;
						}
					}
				}

				//create new local model
				if (!lm)
				{
					nNSS_Model.queryPoint = nearestPoint;

					//update cell pos information (as the nearestPoint may not be inside the same cell as the actual query point!)
					{
						bool inbounds = false;
						int cellPos[3];
						referenceOctree->getTheCellPosWhichIncludesThePoint(&nearestPoint,cellPos,cell.level,inbounds);
						//if the cell is different or the structure has not yet been initialized, we reset it!
						if (	cellPos[0] != nNSS_Model.cellPos[0]
							||	cellPos[1] != nNSS_Model.cellPos[1]
							||	cellPos[2] != nNSS_Model.cellPos[2])
						{
							memcpy(nNSS_Model.cellPos,cellPos,sizeof(int)*3);
							referenceOctree->computeCellCenter(nNSS_Model.cellPos,nNSS_Model.level,nNSS_Model.cellCenter);
							assert(inbounds);
							nNSS_Model.minimalCellsSetToVisit.clear();
							nNSS_Model.pointsInNeighbourhood.clear();
							nNSS_Model.alreadyVisitedNeighbourhoodSize = inbounds ? 0 : 1;
							//nNSS_Model.theNearestPointIndex=0;
						}
					}
					//let's grab the nearest neighbours of the 'nearest point'
					unsigned kNN = 0;
					if (params->useSphericalSearchForLocalModel)
					{
						//we only need to sort neighbours if we want to use the 'reuseExistingLocalModels' optimization
						//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (kNN)!
						kNN = referenceOctree->findNeighborsInASphereStartingFromCell(	nNSS_Model,
																						static_cast<PointCoordinateType>(params->radiusForLocalModel),
																						params->reuseExistingLocalModels);
					}
					else
					{
						kNN = referenceOctree->findNearestNeighborsStartingFromCell(nNSS_Model);
						kNN = std::min(kNN,params->kNNForLocalModel);
					}

					//if there's enough neighbours
					if (kNN >= CC_LOCAL_MODEL_MIN_SIZE[params->localModel])
					{
						DgmOctreeReferenceCloud neighboursCloud(&nNSS_Model.pointsInNeighbourhood,kNN);
						Neighbourhood Z(&neighboursCloud);

						//Neighbours are sorted, so the farthest is at the end. It also gives us
						//an approximation of the model 'size'
						const double& maxSquareDist = nNSS_Model.pointsInNeighbourhood[kNN-1].squareDistd;
						if (maxSquareDist > 0) //DGM: it happens with duplicate points :(
						{
							lm = LocalModel::New(params->localModel,Z,nearestPoint,static_cast<PointCoordinateType>(maxSquareDist));
							if (lm && params->reuseExistingLocalModels)
							{
								//we add the model to the 'existing models' list
								try
								{
									models.push_back(lm);
								}
								catch(std::bad_alloc)
								{
									//not enough memory!
									while (!models.empty())
									{
										delete models.back();
										models.pop_back();
									}
									return false;
								}
							}
						}
						//neighbours->clear();
					}
				}

				//if we have a local model
				if (lm)
				{
					ScalarType distToModel = lm->computeDistanceFromModelToPoint(&nNSS.queryPoint);

					//we take the best estimation between the nearest neighbor and the model!
					//this way we only reduce any potential noise (that would be due to sampling)
					//instead of 'adding' noise if the model is badly shaped
					distPt = std::min(distToNearestPoint,distToModel);

					if (!params->reuseExistingLocalModels)
					{
						//we don't need the local model anymore!
						delete lm;
						lm = 0;
					}
				}
				else
				{
					distPt = distToNearestPoint;
				}
			}
			else if (nNSS.maxSearchSquareDistd > 0)
			{
				distPt = static_cast<ScalarType>(sqrt(nNSS.maxSearchSquareDistd));
			}

			if (params->CPSet)
				params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i),nNSS.theNearestPointIndex);
		}
	
		cell.points->setPointScalarValue(i,distPt);

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	//clear all models for this cell
	while (!models.empty())
	{
		delete models.back();
		models.pop_back();
	}

	return true;
}

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

int DistanceComputationTools::intersectMeshWithOctree(	OctreeAndMeshIntersection* theIntersection,
														uchar octreeLevel,
														GenericProgressCallback* progressCb/*=0*/)
{
	assert(theIntersection);
	DgmOctree* theOctree = theIntersection->theOctree;
	GenericIndexedMesh* theMesh = theIntersection->theMesh;

	//useful variables
	int cellPos[9],maxPos[3],minPos[3],delta[3];
	//current triangle vertices
	const CCVector3* triPoints[3];
	//relative position of each (cell) neighbors relatively to the triangle (plane)
	char pointsPosition[27];

	//cell dimension
	PointCoordinateType cellLength = theIntersection->theOctree->getCellSize(octreeLevel);
	CCVector3 halfCellDimensions(cellLength / 2, cellLength / 2, cellLength / 2);
	std::vector<cellToTest> cellsToTest;

	unsigned cellsToTestCapacity = 1;
	cellsToTest.resize(cellsToTestCapacity);
	unsigned cellsToTestCount = 0;

	//get octree box
	const CCVector3& minBB = theOctree->getOctreeMins();
	//and the number of triangles
	unsigned numberOfTriangles = theMesh->size();

	//for progress notification
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfTriangles);
		char buffer[64];
		sprintf(buffer,"Triangles: %u",numberOfTriangles);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle("Intersect Cloud/Mesh");
		progressCb->start();
	}

	//For each triangle: look for intersecting cells
	theMesh->placeIteratorAtBegining();
	int result = 0;
	for (unsigned n=0; n<numberOfTriangles; ++n)
	{
		//get the positions (in the grid) of each vertex 
		const GenericTriangle* T = theMesh->_getNextTriangle();
		triPoints[0] = T->_getA();
		triPoints[1] = T->_getB();
		triPoints[2] = T->_getC();

		CCVector3 AB = *triPoints[1] - *triPoints[0];
		CCVector3 BC = *triPoints[2] - *triPoints[1];
		CCVector3 CA = *triPoints[0] - *triPoints[2];

		//be sure that the triangle is not degenerate!!!
		if (AB.norm2() > ZERO_TOLERANCE &&
			BC.norm2() > ZERO_TOLERANCE &&
			CA.norm2() > ZERO_TOLERANCE)
		{
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[0],cellPos,octreeLevel);
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[1],cellPos+3,octreeLevel);
			theOctree->getTheCellPosWhichIncludesThePoint(triPoints[2],cellPos+6,octreeLevel);

			//compute the triangle bounding-box
			for (int k=0; k<3; k++)
			{
				int i = std::min(cellPos[3+k],cellPos[6+k]);
				minPos[k] = std::min(cellPos[k],i);
				i = std::max(cellPos[3+k],cellPos[6+k]);
				maxPos[k] = std::max(cellPos[k],i);
			}

			//first cell
			assert(cellsToTestCapacity != 0);
			cellsToTestCount = 1;
			cellToTest* _currentCell = &cellsToTest[0/*cellsToTestCount-1*/];

			for (int k=0; k<3; k++)
			{
				delta[k] = maxPos[k]-minPos[k]+1;
				_currentCell->pos[k] = static_cast<cellToTest::Type>(minPos[k]);
			}
			CCVector3 distanceToOctreeMinBorder = minBB-(*triPoints[0]);

			//compute the triangle normal
			CCVector3 N = AB.cross(BC);

			//max distance (in terms of cell) between the vertices
			int maxSize = std::max(delta[0],delta[1]);
			maxSize = std::max(maxSize,delta[2]);

			//we deduce the smallest bounding 'octree' cell
			//(not a real octree cell in fact as its starting position is anywhere in the grid
			//and it can even 'outbounds' the grid, i.e. currentCell.pos[k]+currentCell.cellSize > octreeLength)
			static const double LOG_2 = log(2.0);
			_currentCell->level = octreeLevel-(maxSize > 1 ? static_cast<uchar>(ceil(log(static_cast<double>(maxSize))/LOG_2)) : 0);
			_currentCell->cellSize = 1 << (octreeLevel-_currentCell->level);

			//now we can (recursively) find the intersecting cells
			while (cellsToTestCount != 0)
			{
				_currentCell = &cellsToTest[--cellsToTestCount];

				//new cells may be written over the actual one
				//so we need to remember its position!
				int currentCellPos[3] = {	static_cast<int>(_currentCell->pos[0]),
											static_cast<int>(_currentCell->pos[1]),
											static_cast<int>(_currentCell->pos[2]) };

				//if we have reached the maximal subdivision level
				if (_currentCell->level == octreeLevel)
				{
					//compute the (absolute) cell center
					theOctree->computeCellCenter(_currentCell->pos,octreeLevel,AB.u);

					//check that the triangle do intersects the cell (box)
					if (CCMiscTools::TriBoxOverlap(AB, halfCellDimensions, triPoints))
					{
						if ((currentCellPos[0] >= theIntersection->minFillIndexes[0] && currentCellPos[0] <= theIntersection->maxFillIndexes[0]) &&
							(currentCellPos[1] >= theIntersection->minFillIndexes[1] && currentCellPos[1] <= theIntersection->maxFillIndexes[1]) &&
							(currentCellPos[2] >= theIntersection->minFillIndexes[2] && currentCellPos[2] <= theIntersection->maxFillIndexes[2]) )
						{
							if (theIntersection->tab)
							{
								int index = (currentCellPos[2]-theIntersection->minFillIndexes[2]) +
											(currentCellPos[1]-theIntersection->minFillIndexes[1]) * theIntersection->dec;
								
								FacesInCellPtr* f = theIntersection->tab[currentCellPos[0]-theIntersection->minFillIndexes[0]] + index;

								if (!(*f))
								{
									(*f) = new FacesInCell();
									(*f)->cellCode = theOctree->generateTruncatedCellCode(_currentCell->pos,octreeLevel);
								}

								//add the triangle to the current 'intersecting triangles' list
								(*f)->push(n);
							}

							if (theIntersection->distanceTransform)
							{
								theIntersection->distanceTransform->setZero(currentCellPos[0]-theIntersection->minFillIndexes[0],
																			currentCellPos[1]-theIntersection->minFillIndexes[1],
																			currentCellPos[2]-theIntersection->minFillIndexes[2]);
							}
						}
					}
				}
				else
				{
					int halfCellSize = _currentCell->cellSize>>1;

					//compute the position of each cell 'neighbors' relatively to the triangle (3*3*3 = 27, including the cell itself)
					{
						char* _pointsPosition = pointsPosition;
						for (int i=0; i<3; ++i)
						{
							AB.x = distanceToOctreeMinBorder.x+PointCoordinateType(currentCellPos[0]+i*halfCellSize)*cellLength;
							for (int j=0; j<3; ++j)
							{
								AB.y = distanceToOctreeMinBorder.y+PointCoordinateType(currentCellPos[1]+j*halfCellSize)*cellLength;
								for (int k=0; k<3; ++k)
								{
									AB.z = distanceToOctreeMinBorder.z+PointCoordinateType(currentCellPos[2]+k*halfCellSize)*cellLength;

									//determine on which side the triangle is
									*_pointsPosition++/*pointsPosition[i*9+j*3+k]*/ = (AB.dot(N) < 0.0 ? -1 : 1);
								}
							}
						}
					}

					//if necessary we enlarge the queue
					if (cellsToTestCount+27 > cellsToTestCapacity)
					{
						cellsToTestCapacity = std::max(cellsToTestCapacity+27,2*cellsToTestCapacity);
						try
						{
							cellsToTest.resize(cellsToTestCapacity);
						}
						catch (std::bad_alloc)
						{
							//out of memory
							return -1;
						}
					}

					//the first new cell will be written over the actual one
					cellToTest* _newCell = &cellsToTest[cellsToTestCount];
					_newCell->level++;
					_newCell->cellSize = halfCellSize;

					//we look at the position of the 8 sub-cubes relatively to the triangle
					for (int i=0; i<2; ++i)
					{
						_newCell->pos[0] = currentCellPos[0] + i*halfCellSize;
						//quick test to determine if the cube is potentially intersecting the triangle's bbox
						if (	static_cast<int>(_newCell->pos[0]) + halfCellSize >= minPos[0]
							&&	static_cast<int>(_newCell->pos[0])                <= maxPos[0] )
						{
							for (int j=0; j<2; ++j)
							{
								_newCell->pos[1] = static_cast<cellToTest::Type>(currentCellPos[1] + j*halfCellSize);
								if (	static_cast<int>(_newCell->pos[1]) + halfCellSize >= minPos[1]
									&&	static_cast<int>(_newCell->pos[1])                <= maxPos[1] )
								{
									for (int k=0;k<2;++k)
									{
										_newCell->pos[2] = static_cast<cellToTest::Type>(currentCellPos[2] + k*halfCellSize);
										if (	static_cast<int>(_newCell->pos[2]) + halfCellSize >= minPos[2]
											&&	static_cast<int>(_newCell->pos[2])                <= maxPos[2] )
										{
											const char* _pointsPosition = pointsPosition + (i*9+j*3+k);
											char sum =	  _pointsPosition[0]  + _pointsPosition[1]  + _pointsPosition[3]
														+ _pointsPosition[4]  + _pointsPosition[9]  + _pointsPosition[10]
														+ _pointsPosition[12] + _pointsPosition[13];

											//if all the sub-cube vertices are not on the same side, then the triangle may intersect the cell
											if (abs(sum) < 8)
											{
												++cellsToTestCount;
												//we make newCell point on next cell in array (we copy current info by the way)
												cellsToTest[cellsToTestCount] = *_newCell;
												_newCell = &cellsToTest[cellsToTestCount];
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if (nProgress && !nProgress->oneStep())
		{
			//cancel by user
			result = -2;
			break;
		}
	}

	if (nProgress)
	{
		delete nProgress;
		nProgress = 0;
	}

	return result;
}

int DistanceComputationTools::computePointCloud2MeshDistanceWithOctree(	OctreeAndMeshIntersection* theIntersection,
																		uchar octreeLevel,
																		bool signedDistances,
																		bool flipTriangleNormals/*=false*/,
																		ScalarType maxSearchDist/*=-1.0*/,
																		GenericProgressCallback* progressCb/*=0*/)
{
	assert(theIntersection);
	assert(!signedDistances || !theIntersection->distanceTransform); //signed distances are not compatible with Distance Transform acceleration

	DgmOctree* theOctree = theIntersection->theOctree;
	GenericIndexedMesh* theMesh = theIntersection->theMesh;

	//dimension of an octree cell
	PointCoordinateType cellLength = theOctree->getCellSize(octreeLevel);

	//get the cell indexes at level "octreeLevel"
	DgmOctree::cellsContainer cellCodesAndIndexes;
	if (!theOctree->getCellCodesAndIndexes(octreeLevel,cellCodesAndIndexes,true))
	{
		//not enough memory
		return -1;
	}

	unsigned numberOfCells = static_cast<unsigned>(cellCodesAndIndexes.size());

	DgmOctree::cellsContainer::const_iterator pCodeAndIndex = cellCodesAndIndexes.begin();
	ReferenceCloud Yk(theOctree->associatedCloud());

	//bounded search
	bool boundedSearch = (maxSearchDist >= 0);
	int maxNeighbourhoodLength = 0; //maximale neighbors search distance (if maxSearchDist is defined)
	if (boundedSearch)
	{
		maxNeighbourhoodLength = static_cast<int>(ceil(maxSearchDist/cellLength + static_cast<ScalarType>((sqrt(2.0)-1.0)/2)));
	}

	//if we only need approximate distances
	if (theIntersection->distanceTransform && !boundedSearch)
	{
		//for each cell
		for (unsigned i=0; i<numberOfCells; ++i,++pCodeAndIndex)
		{
			theOctree->getPointsInCellByCellIndex(&Yk,pCodeAndIndex->theIndex,octreeLevel);

			//get the cell pos
			int pos[3];
			theOctree->getCellPos(pCodeAndIndex->theCode,octreeLevel,pos,true);
			pos[0] -= theIntersection->minFillIndexes[0];
			pos[1] -= theIntersection->minFillIndexes[1];
			pos[2] -= theIntersection->minFillIndexes[2];

			//get the Chamfer distance
			int dist = theIntersection->distanceTransform->getValue(pos);

			//assign the (Chamfer) distance to all points inside this cell
			ScalarType maxRadius = static_cast<ScalarType>(dist)/3 * cellLength;

			unsigned count = Yk.size();
			for (unsigned j=0; j<count; ++j)
				Yk.setPointScalarValue(j,maxRadius);

			//Yk.clear(); //useless
		}

		return 0;
	}

	//otherwise we have to compute the distance from each point to its nearest triangle

	//Progress callback
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfCells);
		char buffer[256];
		sprintf(buffer,"Cells: %u",numberOfCells);
		progressCb->reset();
		progressCb->setInfo(buffer);
		progressCb->setMethodTitle(signedDistances ? "Compute signed distances" : "Compute distances");
		progressCb->start();
	}

	//variables
	std::vector<unsigned> trianglesToTest;
	size_t trianglesToTestCount = 0;
	size_t trianglesToTestCapacity = 0;
	const ScalarType normalSign = static_cast<ScalarType>(flipTriangleNormals ? -1.0 : 1.0);

	//nombre de triangles dans le mesh
	unsigned numberOfTriangles = theMesh->size();

	//acceleration structure
	std::vector<unsigned> processTriangles;
	try
	{
		processTriangles.resize(numberOfTriangles,0);
	}
	catch(std::bad_alloc)
	{
		//otherwise, no big deal, we can do without it!
	}

	//min distance array ('persistent' version to save some memory)
	std::vector<ScalarType> minDists;

	//for each cell
	for (unsigned cellIndex=1; cellIndex<=numberOfCells; ++cellIndex,++pCodeAndIndex) //cellIndex = unique ID for the current cell
	{
		theOctree->getPointsInCellByCellIndex(&Yk,pCodeAndIndex->theIndex,octreeLevel);

		//get cell pos
		int startPos[3];
		theOctree->getCellPos(pCodeAndIndex->theCode,octreeLevel,startPos,true);

		//get the distance to the nearest and farthest boundaries
		int maxDistToBoundaries = 0;
		int distToLowerBorder[3],distToUpperBorder[3];
		for (unsigned k=0; k<3; ++k)
		{
			distToLowerBorder[k] = startPos[k]-theIntersection->minFillIndexes[k];
			maxDistToBoundaries  = std::max(maxDistToBoundaries,distToLowerBorder[k]);
			distToUpperBorder[k] = theIntersection->maxFillIndexes[k]-startPos[k];
			maxDistToBoundaries  = std::max(maxDistToBoundaries,distToUpperBorder[k]);
		}
		int maxDist = maxDistToBoundaries;

		//determine the cell center
		PointCoordinateType cellCenter[3];
		theOctree->computeCellCenter(startPos,octreeLevel,cellCenter);

		//express 'startPos' relatively to the grid borders
		startPos[0] -= theIntersection->minFillIndexes[0];
		startPos[1] -= theIntersection->minFillIndexes[1];
		startPos[2] -= theIntersection->minFillIndexes[2];

		ScalarType maxRadius = 0;
		int dist = 0;
		if (theIntersection->distanceTransform)
		{
			unsigned short dist = theIntersection->distanceTransform->getValue(startPos);
			maxRadius = static_cast<ScalarType>(dist)/3 * cellLength;

			//if (boundedSearch)  //should always be true if we are here!
			{
				if (maxRadius > maxSearchDist)
					maxSearchDist = maxRadius;
			}
		}

		//minDists.clear(); //not necessary 
		unsigned remainingPoints = Yk.size();
		if (minDists.size() < remainingPoints)
		{
			try
			{
				minDists.resize(remainingPoints);
			}
			catch (std::bad_alloc) //out of memory
			{
				if (nProgress)
					delete nProgress;
				//not enough memory
				return -1;
			}
		}

		//for each point, we pre-compute its distance to the nearest cell border
		//(will be handy later)
		for (unsigned j=0; j<remainingPoints; ++j)
		{
			const CCVector3 *tempPt = Yk.getPointPersistentPtr(j);
			//distance to the nearest border = cell size - max distance to the cell center
			minDists[j] = static_cast<ScalarType>(DgmOctree::ComputeMinDistanceToCellBorder(tempPt,cellLength,cellCenter));
		}

		//boundedSearch: compute the accurate distance below 'maxSearchDist'
		//and use the approximate (Chamfer) distances above
		if (boundedSearch)
		{
			//no need to look farther than 'maxNeighbourhoodLength'
			maxDist = std::min(maxDistToBoundaries,maxNeighbourhoodLength);

			for (unsigned j=0; j<remainingPoints; ++j)
				Yk.setPointScalarValue(j,maxSearchDist);
		}

		//let's find the nearest triangles for each point in the neighborhood 'Yk'
		while (remainingPoints != 0 && dist <= maxDist)
		{
			//test the neighbor cells at distance = 'dist'
			//a,b,c,d,e,f are the extents of this neighborhood
			//for the 6 main directions -X,+X,-Y,+Y,-Z,+Z
			int a = std::min(dist,distToLowerBorder[0]);
			int b = std::min(dist,distToUpperBorder[0]);
			int c = std::min(dist,distToLowerBorder[1]);
			int d = std::min(dist,distToUpperBorder[1]);
			int e = std::min(dist,distToLowerBorder[2]);
			int f = std::min(dist,distToUpperBorder[2]);

			int index0 = startPos[0]-a;
			for (int i=-a; i<=b; i++)
			{
				bool imax = (abs(i) == dist);
				const FacesInCellPtr *_tab0 = theIntersection->tab[index0];

				int index = (startPos[1]-c) * static_cast<int>(theIntersection->dec);
				for (int j=-c; j<=d; j++)
				{
					//if i or j is 'maximal'
					if (imax || abs(j) == dist)
					{
						//we must be on the border of the neighborhood
						const FacesInCellPtr *_tab = _tab0+(index+startPos[2]-e);

						for (int k=-e; k<=f; k++)
						{
							//is there a filled cell here?
							if (*_tab)
							{
								const FacesInCellPtr& element = *_tab;
								if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount + element->faceIndexes.size(), 2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
								//let's test all the triangles that intersect this cell
								for (size_t p=0; p<element->faceIndexes.size(); ++p)
								{
									if (!processTriangles.empty())
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//if the triangles has not been processed yet
										if (processTriangles[indexTri] != cellIndex)
										{
											trianglesToTest[trianglesToTestCount++] = indexTri;
											processTriangles[indexTri] = cellIndex;
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
					else //we must go the cube border
					{
						if (e == dist) //'negative' side
						{
							const FacesInCellPtr& element = _tab0[index+startPos[2]-e];
							if (element)
							{
								if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceIndexes.size(),2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
								//let's test all the triangles that intersect this cell
								for (unsigned p=0; p<element->faceIndexes.size(); ++p)
								{
									if (!processTriangles.empty())
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//if the triangles has not been processed yet
										if (processTriangles[indexTri] != cellIndex)
										{
											trianglesToTest[trianglesToTestCount++] = indexTri;
											processTriangles[indexTri] = cellIndex;
										}
									}
									else
									{
										trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
									}
								}
							}
						}

						if (f == dist && dist > 0) //'positive' side
						{
							const FacesInCellPtr& element = _tab0[index+startPos[2]+f];
							if (element)
							{
								if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
								{
									trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceIndexes.size(),2*trianglesToTestCount);
									trianglesToTest.resize(trianglesToTestCapacity);
								}
								//let's test all the triangles that intersect this cell
								for (unsigned p=0; p<element->faceIndexes.size(); ++p)
								{
									if (!processTriangles.empty())
									{
										const unsigned& indexTri = element->faceIndexes[p];
										//if the triangles has not been processed yet
										if (processTriangles[indexTri] != cellIndex)
										{
											trianglesToTest[trianglesToTestCount++] = indexTri;
											processTriangles[indexTri] = cellIndex;
										}
									}
									else
									{
										trianglesToTest[trianglesToTestCount++] = element->faceIndexes[p];
									}
								}
							}
						}
					}

					index += static_cast<int>(theIntersection->dec);
				}

				index0++;
			}

			//thanks to 'processTriangles' we shouldn't have tested the triangle yet
			//for smaller neighborhood radii
			bool firstComparisonDone = (trianglesToTestCount != 0);

			//for each triangle
			while (trianglesToTestCount != 0)
			{
				//we query the vertex coordinates (pointers to)
				GenericTriangle *tempTri = theMesh->_getTriangle(trianglesToTest[--trianglesToTestCount]);

				//for each point inside the current cell
				Yk.placeIteratorAtBegining();
				if (signedDistances)
				{
					//we have to use absolute distances
					for (unsigned j=0; j<remainingPoints; ++j)
					{
						//compute the distance to the triangle
						ScalarType dPTri = computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),tempTri,true);
						//keep it if it's smaller
						ScalarType min_d = Yk.getCurrentPointScalarValue();
						if (!ScalarField::ValidValue(min_d) || min_d*min_d > dPTri*dPTri)
							Yk.setCurrentPointScalarValue(normalSign*dPTri);
						Yk.forwardIterator();
					}
				}
				else //squared distances
				{
					for (unsigned j=0;j<remainingPoints;++j)
					{
						//compute the (SQUARED) distance to the triangle
						ScalarType dPTri = computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),tempTri,false);
						//keep it if it's smaller
						ScalarType min_d = Yk.getCurrentPointScalarValue();
						if (!ScalarField::ValidValue(min_d) || dPTri < min_d)
							Yk.setCurrentPointScalarValue(dPTri);
						Yk.forwardIterator();
					}
				}
			}

			//we can 'remove' all the eligible points at the current neighborhood radius
			if (firstComparisonDone)
			{
				for (unsigned j=0; j<remainingPoints; )
				{
					//eligibility distance
					ScalarType eligibleDist = minDists[j] + maxRadius;
					ScalarType dPTri = Yk.getPointScalarValue(j);
					if (signedDistances)
					{
						//need to get the square distance in all cases
						dPTri *= dPTri;
					}
					if (dPTri <= eligibleDist*eligibleDist)
					{
						//remove this point
						Yk.removePointGlobalIndex(j);
						//and do the same for the 'minDists' array! (see ReferenceCloud::removeCurrentPointGlobalIndex)
						assert(remainingPoints != 0);
						minDists[j] = minDists[--remainingPoints];
						//minDists.pop_back();
					}
					else
					{
						++j;
					}
				}
			}

			++dist;
			maxRadius += static_cast<ScalarType>(cellLength);
		}

		//Yk.clear(); //pas necessaire

		if (nProgress && !nProgress->oneStep())
			break;
	}

	if (nProgress)
	{
		delete nProgress;
		nProgress = 0;
	}

	return 0;
}

#ifdef ENABLE_CLOUD2MESH_DIST_MT

#include <QtCore>
#include <QApplication>
#include <QtConcurrentMap>

/*** MULTI THREADING WRAPPER ***/

static DgmOctree* s_octree_MT = 0;
static NormalizedProgress* s_normProgressCb_MT = 0;
static bool s_cellFunc_MT_success = true;
static unsigned char s_octreeLevel_MT = 0;
static OctreeAndMeshIntersection* s_theIntersection_MT = 0;
static bool s_signedDistances_MT = true;
static ScalarType s_normalSign_MT = 1.0f;

//'processTriangles' mechanism (based on bit mask)
#include <QtCore/QBitArray>
static std::vector<QBitArray*> s_bitArrayPool_MT;
static bool s_useBitArrays_MT = true;
static QMutex s_currentBitMaskMutex;

void cloudMeshDistCellFunc_MT(const DgmOctree::IndexAndCode& desc)
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

	ScalarType* minDists = new ScalarType[remainingPoints];
	if (!minDists) //not enough memory?
	{
		s_cellFunc_MT_success = false;
		return;
	}

	//on recupere la position de la cellule (dans startPos)
	int startPos[3];
	s_octree_MT->getCellPos(desc.theCode,s_octreeLevel_MT,startPos,true);

	//on en deduit le symetrique ainsi que la distance au bord de la grille le plus eloigne (maxDistToBoundaries)
	int maxDistToBoundaries = 0;
	int distToLowerBorder[3],distToUpperBorder[3];
	for (unsigned k=0;k<3;++k)
	{
		distToLowerBorder[k] = startPos[k]-s_theIntersection_MT->minFillIndexes[k];
		maxDistToBoundaries=std::max(maxDistToBoundaries,distToLowerBorder[k]);
		distToUpperBorder[k] = s_theIntersection_MT->maxFillIndexes[k]-startPos[k];
		maxDistToBoundaries=std::max(maxDistToBoundaries,distToUpperBorder[k]);
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
	size_t trianglesToTestCount=0;
	size_t trianglesToTestCapacity=0;

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
	//cela nous permettra de recalculer plus rapidement la distance d'eligibilite
	//du triangle le plus proche
	Yk.placeIteratorAtBegining();
	for (unsigned j=0;j<remainingPoints;++j)
	{
		//coordonnees du point courant
		const CCVector3 *tempPt = Yk.getCurrentPointCoordinates();
		//distance du bord le plus proche = taille de la cellule - distance la plus grande par rapport au centre de la cellule
		//minDists.push_back(cellLength*0.5-DgmOctree::computeMaxDistanceToCellCenter(tempPt,cellCenter));
		minDists[j] = DgmOctree::ComputeMinDistanceToCellBorder(tempPt,cellLength,cellCenter);
		Yk.forwardIterator();
	}

	//initialisation de la recurrence
	ScalarType maxRadius=0;
	int dist=0;

	//on va essayer de trouver les triangles les plus proches de chaque point du "voisinage" Yk
	while (remainingPoints>0 && dist<=maxDist)
	{
		//on va tester les cellules voisines a une distance "dist"
		//a,b,c,d,e,f representent l'extension spatiale du voisinage INCLUS DANS LA GRILLE 3D
		//selon les 6 directions -X,+X,-Y,+Y,-Z,+Z
		int a = std::min(dist,distToLowerBorder[0]);
		int b = std::min(dist,distToUpperBorder[0]);
		int c = std::min(dist,distToLowerBorder[1]);
		int d = std::min(dist,distToUpperBorder[1]);
		int e = std::min(dist,distToLowerBorder[2]);
		int f = std::min(dist,distToUpperBorder[2]);

		int index0 = startPos[0]-a;
		for (int i=-a;i<=b;i++)
		{
			bool imax = (abs(i)==dist);
			FacesInCellPtr *_tab0 = s_theIntersection_MT->tab[index0];

			//index0 = (startPos[0]=i)*dec2;
			int index = (startPos[1]-c)*(int)s_theIntersection_MT->dec;
			for (int j=-c;j<=d;j++)
			{
				//index = index0+(startPos[1]+j)*dec1;

				//si i ou j est maximal
				if (imax || abs(j)==dist)
				{
					//on est forcement sur le bord du voisinage
					FacesInCellPtr *_tab = _tab0+(index+startPos[2]-e);

					for (int k=-e;k<=f;k++)
					{
						//element correspondant
						if (*_tab)
						{
							const FacesInCellPtr& element = *_tab;
							if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceIndexes.size(),2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceIndexes.size();++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas deja ete insere
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
					if (e==dist) //cote negatif
					{
						//index correspondant
						const FacesInCellPtr& element = _tab0[index+startPos[2]-e];

						if (element)
						{
							if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceIndexes.size(),2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceIndexes.size();++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas deja ete insere
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

					if (f==dist && dist>0) //cote positif
					{
						//index correspondant
						const FacesInCellPtr& element = _tab0[index+startPos[2]+f];

						if (element)
						{
							if (trianglesToTestCount + element->faceIndexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount+element->faceIndexes.size(),2*trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//pour chaque triangle intersectant la cellule en cours
							for (unsigned p=0;p<element->faceIndexes.size();++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = element->faceIndexes[p];
									//si le triangle n'a pas deja ete insere
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
			//on recupere les coordonnees de ses sommets (references)
			CCLib::SimpleTriangle tri;
			s_theIntersection_MT->theMesh->getTriangleSummits(trianglesToTest[--trianglesToTestCount],tri.A,tri.B,tri.C);

			//pour chaque point dans la cellule
			Yk.placeIteratorAtBegining();
			if (s_signedDistances_MT)
			{
				for (unsigned j=0;j<remainingPoints;++j)
				{
					//on calcule la distance point/triangle
					ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),&tri,true);
					//si elle est plus petite que la distance actuelle du point, on remplace
					ScalarType min_d = Yk.getCurrentPointScalarValue();
					if (!ScalarField::ValidValue(min_d) || min_d*min_d > dPTri*dPTri)
						Yk.setCurrentPointScalarValue(s_normalSign_MT*dPTri);
					Yk.forwardIterator();
				}
			}
			else
			{
				for (unsigned j=0;j<remainingPoints;++j)
				{
					//on calcule la distance point/triangle (warning: we already get the squared distance in this case!)
					ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getCurrentPointCoordinates(),&tri,false);
					//si elle est plus petite que la distance actuelle du point, on remplace
					ScalarType min_d = Yk.getCurrentPointScalarValue();
					if (!ScalarField::ValidValue(min_d) || dPTri<min_d)
						Yk.setCurrentPointScalarValue(dPTri);
					Yk.forwardIterator();
				}
			}
		}

		//maintenant on retire tous les points "elus" lors de ce tour
		if (firstComparisonDone)
		{
			Yk.placeIteratorAtBegining();
			for (unsigned j=0;j<remainingPoints;++j)
			{
				//distance d'eligibilite
				ScalarType eligibleDist = minDists[j]+maxRadius;
				ScalarType dPTri = Yk.getCurrentPointScalarValue();
				if (s_signedDistances_MT)
					dPTri*=dPTri;
				if (dPTri <= eligibleDist*eligibleDist)
				{
					//on supprime le point courant
					Yk.removeCurrentPointGlobalIndex();
					//on applique l'operation equivalente (voir ReferenceCloud::removeCurrentPointGlobalIndex)
					//au tableau donnant la distance minimale par rapport a la cellule
					assert(remainingPoints != 0);
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

//Calcul de la distance signee entre un maillage et un nuage, accelere par la structure "OctreeAndMeshIntersection" (multi-threaded)
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

	unsigned numberOfCells = (unsigned)cellsDescs.size();

	//Progress callback
	NormalizedProgress* nProgress = 0;
	if (progressCb)
	{
		nProgress = new NormalizedProgress(progressCb,numberOfCells);
		char buffer[256];
		sprintf(buffer,"Cells=%u",numberOfCells);
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

//convert all 'distances' (squared in fact) to their square root
inline void applySqrtToPointDist(const CCVector3 &aPoint, ScalarType& aScalarValue)
{
	if (ScalarField::ValidValue(aScalarValue))
		aScalarValue = sqrt(aScalarValue);
}

int DistanceComputationTools::computePointCloud2MeshDistance(	GenericIndexedCloudPersist* pointCloud,
																GenericIndexedMesh* theMesh,
																uchar octreeLevel,
																ScalarType maxSearchDist,
																bool useDistanceMap/*=false*/,
																bool signedDistances/*=false*/,
																bool flipNormals/*=false*/,
																bool multiThread/*=true*/,
																GenericProgressCallback* progressCb/*=0*/,
																DgmOctree* cloudOctree/*=0*/)
{
	assert(pointCloud && theMesh);

	//tests preliminaires sur les variables d'entree
	if (pointCloud->size() == 0 || theMesh->size() == 0)
		return -2;

	//signed distances are incompatible with Chamfer approximation
	if (signedDistances)
		useDistanceMap = false;

	//on regarde si la boite englobante du nuage et du maillage coinc\EFdent
	CCVector3 cloudMinBB,cloudMaxBB,meshMinBB,meshMaxBB,minBB,maxBB,minCubifiedBB,maxCubifiedBB;
	pointCloud->getBoundingBox(cloudMinBB.u,cloudMaxBB.u);
	theMesh->getBoundingBox(meshMinBB.u,meshMaxBB.u);

	//on calcule les limites de la boite englobante maximale
	{
		for (uchar k=0;k<3;++k)
		{
			minBB.u[k] = std::min(meshMinBB.u[k],cloudMinBB.u[k]);
			maxBB.u[k] = std::max(meshMaxBB.u[k],cloudMaxBB.u[k]);
		}
	}

	//on calcule les dimensions de l'octree correspondant
	minCubifiedBB = minBB;
	maxCubifiedBB = maxBB;
	CCMiscTools::MakeMinAndMaxCubical(minCubifiedBB,maxCubifiedBB);

	//calcul de la structure octree
	DgmOctree* theOctree = cloudOctree;
	bool wrongOctreeDimensions = false;
	if (!theOctree)
	{
		theOctree = new DgmOctree(pointCloud);
		wrongOctreeDimensions = true;
	}
	else
	{
		//on verifie que l'octree fourni en entree a les bonnes dimensions
		const CCVector3& theOctreeMins = theOctree->getOctreeMins();
		const CCVector3& theOctreeMaxs = theOctree->getOctreeMaxs();
		for (uchar k=0; k<3; ++k)
		{
			if (	theOctreeMins.u[k] != minCubifiedBB.u[k]
				||	theOctreeMaxs.u[k] != maxCubifiedBB.u[k] )
			{
				wrongOctreeDimensions = true;
				break;
			}
		}
	}

	//on doit re(calculer) l'octree
	if (wrongOctreeDimensions)
	{
		if (theOctree->build(minCubifiedBB,maxCubifiedBB,&cloudMinBB,&cloudMaxBB,progressCb) < 1)
		{
			if (!cloudOctree)
				delete theOctree;
			return -36;
		}
	}

	OctreeAndMeshIntersection theIntersection;

	//we deduce grid cell size very simply (as bbox has been "cubified")
	PointCoordinateType cellSize = (maxCubifiedBB.x-minCubifiedBB.x) / (1<<octreeLevel);
	//we compute grid occupancy ...
	//... and we deduce array dimensions
	unsigned tabSizes[3];
	{
		for (uchar k=0;k<3;++k)
		{
			theIntersection.minFillIndexes[k] = static_cast<int>(floor((minBB.u[k]-minCubifiedBB.u[k])/cellSize));
			theIntersection.maxFillIndexes[k] = static_cast<int>(floor((maxBB.u[k]-minCubifiedBB.u[k])/cellSize));
			tabSizes[k] = static_cast<unsigned>(theIntersection.maxFillIndexes[k]-theIntersection.minFillIndexes[k]+1);
		}
	}

	theIntersection.theOctree = theOctree;
	theIntersection.theMesh = theMesh;
	theIntersection.dec = tabSizes[2];
	theIntersection.sliceNumber = tabSizes[0];
	theIntersection.sliceSize = tabSizes[1]*tabSizes[2];

	bool boundedSearch = (maxSearchDist >= 0);
	multiThread &= (!boundedSearch); //MT doesn't support boundedSearch yet!
	if (!useDistanceMap || boundedSearch)
	{
		//structure contenant pour chaque cellule de la grille 3D
		//la liste des triangles intersectants
		//Rq : tableau a 2 "dimensions" pour eviter les blocs de memoire trop gros
		theIntersection.tab = new FacesInCellPtr*[theIntersection.sliceNumber];
		if (!theIntersection.tab)
		{
			if (!cloudOctree)
				delete theIntersection.theOctree;
			return -4;
		}
		memset(theIntersection.tab,0,theIntersection.sliceNumber*sizeof(FacesInCellPtr*));

		//plutot que d'instancier d'un bloc la grille 3D, on instancie chaque tranche separemment
		//comme \E7a on a plus de chance de trouver des segments de memoire contigue suffisamment grands
		for (unsigned i=0;i<theIntersection.sliceNumber;++i)
		{
			theIntersection.tab[i] = new FacesInCellPtr[theIntersection.sliceSize];
			if (!theIntersection.tab[i])
			{
				if (!cloudOctree)
					delete theIntersection.theOctree;
				return -4;
			}
			memset(theIntersection.tab[i],0,theIntersection.sliceSize*sizeof(FacesInCellPtr*));
		}
	}
	else
	{
		//we only compute approximate (Chamfer) distances
		multiThread = false; //not necessary/supported
	}

	//si l'utilisateur (ou la configuration nuage/maillage) necessite l'utilisation d'une transformee
	//de distance de type Chanfrein.
	if (useDistanceMap)
	{
		theIntersection.distanceTransform = new ChamferDistanceTransform(tabSizes[0],tabSizes[1],tabSizes[2]);
		if ( !theIntersection.distanceTransform || !theIntersection.distanceTransform->init())
		{
			if (!cloudOctree)
				delete theIntersection.theOctree;
			return -5;
		}
	}

	//ON INTERSECTE L'OCTREE AVEC LE MAILLAGE
	int result = intersectMeshWithOctree(&theIntersection,octreeLevel,progressCb);
	if (result < 0)
	{
		if (!cloudOctree)
			delete theIntersection.theOctree;
		return -6;
	}

	//raz des distances
	pointCloud->enableScalarField();
	pointCloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

    //acceleration par caclul d'une transformee de distance
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
// Inspired from documents and code by:
// David Eberly
// Geometric Tools, LLC
// http://www.geometrictools.com/
ScalarType DistanceComputationTools::computePoint2TriangleDistance(const CCVector3* P, const GenericTriangle* theTriangle, bool signedDist)
{
    assert(P && theTriangle);

	const CCVector3* A = theTriangle->_getA();
	const CCVector3* B = theTriangle->_getB();
	const CCVector3* C = theTriangle->_getC();

	//we do all computations with double precision, otherwise
	//some triangles with sharp angles will give very poor results.
	CCVector3d AP(P->x-A->x,P->y-A->y,P->z-A->z);
	CCVector3d AB(B->x-A->x,B->y-A->y,B->z-A->z);
	CCVector3d AC(C->x-A->x,C->y-A->y,C->z-A->z);

	double fSqrDist;
	{
		double fA00 = AB.dot(AB);
		double fA11 = AC.dot(AC);
		double fA01 = AB.dot(AC);
		double fB0 = -AP.dot(AB);
		double fB1 = -AP.dot(AC);
		fSqrDist = /*double fC =*/ AP.dot(AP); //DGM: in fact, fSqrDist is always equal to fC + something
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
		ScalarType d = (ScalarType)sqrt(fabs(fSqrDist)); //fabs --> sometimes we get near-0 negative values!

		//triangle normal
		CCVector3d N = AB.cross(AC);

		//we test the sign of the cross product of the triangle normal and the vector AP
		return (AP.dot(N) < 0 ? -d : d);
	}
	else
	{
		return (ScalarType)fabs(fSqrDist); //fabs --> sometimes we get near-0 negative values!
	}
}

ScalarType DistanceComputationTools::computePoint2PlaneDistance(const CCVector3* P,
																const PointCoordinateType* planeEquation)
{
	//point to plane distance: d = fabs(a0*x+a1*y+a2*z-a3)/sqrt(a0^2+a1^2+a2^2)
	assert(fabs(CCVector3::vnorm(planeEquation) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	return static_cast<ScalarType>((CCVector3::vdot(P->u,planeEquation)-planeEquation[3])/*/CCVector3::vnorm(planeEquation)*/); //norm == 1.0!
}

ScalarType DistanceComputationTools::computeCloud2PlaneDistanceRMS(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
    assert(cloud && planeEquation);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = fabs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
        return NAN_VALUE;
	assert(fabs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	double dSumSq = 0.0;

	//compute deviations
	cloud->placeIteratorAtBegining();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		double d = static_cast<double>(CCVector3::vdot(P->u,planeEquation)-planeEquation[3])/*/norm*/; //norm == 1.0
		
		dSumSq += d*d;
	}

	return (ScalarType)sqrt(dSumSq/(double)count);
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation,
																	float percent)
{
    assert(cloud && planeEquation);
	assert(percent < 1.0f);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = fabs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
        return NAN_VALUE;
	assert(fabs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max @ 'percent'% (to avoid outliers)
	std::vector<PointCoordinateType> tail;
	size_t tailSize = static_cast<size_t>(ceil(static_cast<float>(count) * percent));
	tail.resize(tailSize);

	//compute deviations
	cloud->placeIteratorAtBegining();
	size_t pos = 0;
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = fabs(CCVector3::vdot(P->u,planeEquation)-planeEquation[3])/*/norm*/; //norm == 1.0

		if (pos < tailSize)
		{
			tail[pos++] = d;
		}
		else if (tail.back() < d)
		{
			tail.back() = d;
		}

		//search the max element of the tail
		size_t maxPos = pos-1;
		if (maxPos != 0)
		{
			size_t maxIndex = maxPos;
			for (size_t j=0; j<maxPos; ++j)
				if (tail[j] < tail[maxIndex])
					maxIndex = j;
			//and put it to the back!
			if (maxPos != maxIndex)
				std::swap(tail[maxIndex],tail[maxPos]);
		}
	}

	return static_cast<ScalarType>(tail.back());
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneMaxDistance(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);

	//point count
	unsigned count = cloud->size();
	if (count == 0)
		return 0;

	//point to plane distance: d = fabs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
		return NAN_VALUE;
	assert(fabs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max distance
	PointCoordinateType maxDist = 0;
	
	cloud->placeIteratorAtBegining();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = fabs(CCVector3::vdot(P->u,planeEquation)-planeEquation[3])/*/norm*/; //norm == 1.0
		maxDist = std::max(d,maxDist);
	}

	return static_cast<ScalarType>(maxDist);
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneDistance(CCLib::GenericCloud* cloud,
																const PointCoordinateType* planeEquation,
																ERROR_MEASURES measureType)
{
	switch (measureType)
	{
	case RMS:
		return CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud,planeEquation);

	case MAX_DIST_68_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.32f);
	case MAX_DIST_95_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.05f);
	case MAX_DIST_99_PERCENT:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneRobustMax(cloud,planeEquation,0.01f);
	
	case MAX_DIST:
		return CCLib::DistanceComputationTools::ComputeCloud2PlaneMaxDistance(cloud,planeEquation);

	default:
		assert(false);
		return -1.0;
	}
}

bool DistanceComputationTools::computeGeodesicDistances(GenericIndexedCloudPersist* cloud, unsigned seedPointIndex, uchar octreeLevel, GenericProgressCallback* progressCb)
{
	assert(cloud);

	unsigned n = cloud->size();
	if (n == 0 || seedPointIndex >= n)
		return false;

	cloud->enableScalarField();
	cloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

	DgmOctree* theOctree = new DgmOctree(cloud);
	if (theOctree->build(progressCb) < 1)
	{
		delete theOctree;
		return false;
	}

	FastMarchingForPropagation fm;
	if (fm.init(cloud,theOctree,octreeLevel,true) < 0)
	{
		delete theOctree;
		return false;
	}

	//on cherche la cellule de l'octree qui englobe le "seedPoint"
	int cellPos[3];
	theOctree->getTheCellPosWhichIncludesThePoint(cloud->getPoint(seedPointIndex),cellPos,octreeLevel);
	fm.setSeedCell(cellPos);

	bool result = false;
	if (fm.propagate() >= 0)
		result = fm.setPropagationTimingsAsDistances();

	delete theOctree;
	theOctree = 0;

	return result;
}

int DistanceComputationTools::diff(	GenericIndexedCloudPersist* comparedCloud,
									GenericIndexedCloudPersist* referenceCloud,
									GenericProgressCallback* progressCb)
{
	if (!comparedCloud || !referenceCloud)
		return -1;

	unsigned nA = comparedCloud->size();
	if (nA == 0)
		return -2;

	//Reference cloud to store closest point set
	ReferenceCloud A_in_B(referenceCloud);

	Cloud2CloudDistanceComputationParams params;
	params.octreeLevel = DgmOctree::MAX_OCTREE_LEVEL-1;
	params.CPSet = &A_in_B;

	int result = computeHausdorffDistance(comparedCloud,referenceCloud,params,progressCb);
	if (result < 0)
		return -3;

	for (unsigned i=0; i<nA; ++i)
	{
		ScalarType dA = comparedCloud->getPointScalarValue(i);
		ScalarType dB = A_in_B.getPointScalarValue(i);

		//handle invalid values
		comparedCloud->setPointScalarValue(i,ScalarField::ValidValue(dA) && ScalarField::ValidValue(dB) ? dA-dB : NAN_VALUE);
	}

	return 0;
}

int DistanceComputationTools::computeChamferDistanceBetweenTwoClouds(	CC_CHAMFER_DISTANCE_TYPE cType,
																		GenericIndexedCloudPersist* comparedCloud,
																		GenericIndexedCloudPersist* referenceCloud,
																		uchar octreeLevel,
																		PointCoordinateType maxSearchDist/*=-PC_ONE*/,
																		GenericProgressCallback* progressCb/*=0*/,
																		DgmOctree* compOctree/*=0*/,
																		DgmOctree* refOctree/*=0*/)
{
	if (!comparedCloud || !referenceCloud)
		return -1;
	if (octreeLevel < 1 || octreeLevel > DgmOctree::MAX_OCTREE_LEVEL)
		return -2;

	//compute octrees with the same bounding-box
	DgmOctree *octreeA = compOctree, *octreeB = refOctree;
	if (synchronizeOctrees(comparedCloud,referenceCloud,octreeA,octreeB,maxSearchDist,progressCb) != SYNCHRONIZED)
		return -3;

	const int* minIndexesA = octreeA->getMinFillIndexes(octreeLevel);
	const int* maxIndexesA = octreeA->getMaxFillIndexes(octreeLevel);
	const int* minIndexesB = octreeB->getMinFillIndexes(octreeLevel);
	const int* maxIndexesB = octreeB->getMaxFillIndexes(octreeLevel);

	int minIndexes[3] = {	std::min(minIndexesA[0],minIndexesB[0]),
							std::min(minIndexesA[1],minIndexesB[1]),
							std::min(minIndexesA[2],minIndexesB[2]) };
	int maxIndexes[3] = {	std::max(maxIndexesA[0],maxIndexesB[0]),
							std::max(maxIndexesA[1],maxIndexesB[1]),
							std::max(maxIndexesA[2],maxIndexesB[2]) };
	unsigned short boxSize[3] = {	static_cast<unsigned short>(maxIndexes[0] - minIndexes[0]+1),
									static_cast<unsigned short>(maxIndexes[1] - minIndexes[1]+1),
									static_cast<unsigned short>(maxIndexes[2] - minIndexes[2]+1) };

	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return -1;
	}
	if (maxSearchDist >= 0)
	{
		//if maxSearchDist is defined, we might skip some points
		//so we set a default distance for all of them
		const ScalarType resetValue = static_cast<ScalarType>(maxSearchDist);
		for (unsigned i=0; i<comparedCloud->size(); ++i)
			comparedCloud->setPointScalarValue(i,resetValue);
	}

	int result = 0;

	//instantiate the Chamfer grid
	ChamferDistanceTransform chamferGrid(boxSize[0],boxSize[1],boxSize[2]);
	if (chamferGrid.init())
	{
		//project the (filled) cells of octree B in the Chamfer grid
		{
			DgmOctree::cellCodesContainer theCodes;
			octreeB->getCellCodes(octreeLevel,theCodes,true);

			while (!theCodes.empty())
			{
				DgmOctree::OctreeCellCodeType theCode = theCodes.back();
				theCodes.pop_back();
				int pos[3];
				octreeB->getCellPos(theCode,octreeLevel,pos,true);
				pos[0] -= minIndexes[0];
				pos[1] -= minIndexes[1];
				pos[2] -= minIndexes[2];
				chamferGrid.setZero(pos);
			}
		}

		//propagate the Chamfer distance over the grid
		chamferGrid.propagateDistance(cType,progressCb);

		//eventually get the (Chamfer) distance for each cell of octree A
		//and assign it to the points inside
		ScalarType cellSize = static_cast<ScalarType>(octreeA->getCellSize(octreeLevel));
		if (cType == CHAMFER_345)
			cellSize /= 3;

		DgmOctree::cellIndexesContainer theIndexes;
		if (!octreeA->getCellIndexes(octreeLevel,theIndexes))
		{
			//not enough memory
			if (!compOctree)
				delete octreeA;
			if (!refOctree)
				delete octreeB;
			return -5;
		}

		int maxDi = 0;
		ReferenceCloud Yk(octreeA->associatedCloud());

		while (!theIndexes.empty())
		{
			unsigned theIndex = theIndexes.back();
			theIndexes.pop_back();

			int pos[3];
			octreeA->getCellPos(octreeA->getCellCode(theIndex),octreeLevel,pos,false);
			pos[0] -= minIndexes[0];
			pos[1] -= minIndexes[1];
			pos[2] -= minIndexes[2];
			int di = static_cast<int>(chamferGrid.getValue(pos));
			if (di > maxDi)
				maxDi = di;
			ScalarType d = static_cast<ScalarType>(di) * cellSize;
			
			//the maximum distance is 'maxSearchDist' (if defined)
			if (maxSearchDist < 0 || d < maxSearchDist)
			{
				octreeA->getPointsInCellByCellIndex(&Yk,theIndex,octreeLevel);
				for (unsigned j=0; j<Yk.size(); ++j)
					Yk.setPointScalarValue(j,d);
			}
		}

		if (cType == CHAMFER_345)
			maxDi /= 3;

		result = maxDi;
	}
	else //chamferGrid init failed
	{
		result = -4;
	}

	if (!compOctree)
	{
		delete octreeA;
		octreeA = 0;
	}
	if (!refOctree)
	{
		delete octreeB;
		octreeB = 0;
	}

	return result;
}

PointCoordinateType DistanceComputationTools::ComputeSquareDistToSegment(const CCVector2& P,
																		 const CCVector2& A,
																		 const CCVector2& B,
																		 bool onlyOrthogonal/*=false*/)
{
	CCVector2 AP = P-A;
	CCVector2 AB = B-A;
	PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
	if (dot < 0)
	{
		return onlyOrthogonal ? -PC_ONE : AP.norm2();
	}
	else
	{
		PointCoordinateType squareLengthAB = AB.norm2();
		if (dot > squareLengthAB)
		{
			return onlyOrthogonal ? -PC_ONE : (P-B).norm2();
		}
		else
		{
			CCVector2 HP = AP - AB * (dot / squareLengthAB);
			return HP.norm2();
		}
	}
}