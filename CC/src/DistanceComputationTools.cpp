//##########################################################################
//#																		   #
//#								  CCLIB									   #
//#																		   #
//#	 This program is free software; you can redistribute it and/or modify  #
//#	 it under the terms of the GNU Library General Public License as	   #
//#	 published by the Free Software Foundation; version 2 or later of the  #
//#	 License.															   #
//#																		   #
//#	 This program is distributed in the hope that it will be useful,	   #
//#	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   #
//#	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the		   #
//#	 GNU General Public License for more details.						   #
//#																		   #
//#			 COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)			   #
//#																		   #
//##########################################################################

#include <DistanceComputationTools.h>

//local
#include <DgmOctreeReferenceCloud.h>
#include <FastMarchingForPropagation.h>
#include <LocalModel.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <SaitoSquaredDistanceTransform.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <SimpleTriangle.h>
#include <SquareMatrix.h>
#include <Polyline.h>

//system
#include <algorithm>
#include <cassert>

#ifdef USE_QT
#ifndef CC_DEBUG
//enables multi-threading handling
#define ENABLE_CLOUD2MESH_DIST_MT
#endif
#endif

namespace CCLib
{

	//! List of triangles (indexes)
	struct TriangleList
	{
		//! Triangles indexes
		std::vector<unsigned> indexes;

		//! Adds a triangle index
		/** \return success
		**/
		inline bool push(unsigned index)
		{
			try
			{
				indexes.push_back(index);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}
			return true;
		}
	};

	//! Internal structure used by DistanceComputationTools::computeCloud2MeshDistance
	struct OctreeAndMeshIntersection
	{
	public:

		//! Octree structure
		DgmOctree* octree;
		//! Mesh
		GenericIndexedMesh* mesh;
		//! Distance transform
		SaitoSquaredDistanceTransform* distanceTransform;

		//! Grid occupancy of mesh (minimum indexes for each dimension)
		Tuple3i minFillIndexes;
		//! Grid occupancy of mesh (maximum indexes for each dimension)
		Tuple3i maxFillIndexes;

		//! Array of FacesInCellPtr structures
		Grid3D<TriangleList*> perCellTriangleList;

		//! Default constructor
		OctreeAndMeshIntersection()
			: octree(nullptr)
			, mesh(nullptr)
			, distanceTransform(nullptr)
			, minFillIndexes(0, 0, 0)
			, maxFillIndexes(0, 0, 0)
		{}

		//! Destructor
		~OctreeAndMeshIntersection()
		{
			if (perCellTriangleList.isInitialized())
			{
				TriangleList** data = perCellTriangleList.data();
				for (std::size_t i = 0; i < perCellTriangleList.totalCellCount(); ++i, ++data)
				{
					if (*data)
						delete (*data);
				}
			}

			if (distanceTransform)
			{
				delete distanceTransform;
				distanceTransform = nullptr;
			}
		}
	};
} //namespace CCLib

using namespace CCLib;

bool DistanceComputationTools::MultiThreadSupport()
{
#ifdef ENABLE_CLOUD2MESH_DIST_MT
	return true;
#else
	return false;
#endif
}

int DistanceComputationTools::computeCloud2CloudDistance(	GenericIndexedCloudPersist* comparedCloud,
															GenericIndexedCloudPersist* referenceCloud,
															Cloud2CloudDistanceComputationParams& params,
															GenericProgressCallback* progressCb/*=0*/,
															DgmOctree* compOctree/*=0*/,
															DgmOctree* refOctree/*=0*/)
{
	assert(comparedCloud && referenceCloud);
	if (!comparedCloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (comparedCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (!referenceCloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}

	if (referenceCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCECLOUD;
	}

	if (params.CPSet && params.maxSearchDist > 0)
	{
		//we can't use a 'max search distance' criterion if the "Closest Point Set" is requested
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_CANT_USE_MAX_SEARCH_DIST_AND_CLOSEST_POINT_SET;
	}

	//we spatially 'synchronize' the octrees
	DgmOctree *comparedOctree = compOctree;
	DgmOctree *referenceOctree = refOctree;
	SOReturnCode soCode = synchronizeOctrees(	comparedCloud,
												referenceCloud,
												comparedOctree,
												referenceOctree,
												params.maxSearchDist,
												progressCb);
	
	if (soCode != SYNCHRONIZED && soCode != DISJOINT)
	{
		//not enough memory (or invalid input)
		return DISTANCE_COMPUTATION_RESULTS::ERROR_SYNCHRONIZE_OCTREES_FAILURE;
	}

	//we 'enable' a scalar field  (if it is not already done) to store resulting distances
	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
	}

	//internally we don't use the maxSearchDist parameters as is, but the square of it
	double maxSearchSquareDistd = params.maxSearchDist <= 0 ? 0 : static_cast<double>(params.maxSearchDist) * params.maxSearchDist;

	//closest point set
	if (params.CPSet)
	{
		assert(maxSearchSquareDistd <= 0);
		
		if (!params.CPSet->resize(comparedCloud->size()))
		{
			//not enough memory
			if (comparedOctree && !compOctree)
				delete comparedOctree;
			if (referenceOctree && !refOctree)
				delete referenceOctree;
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}
	}

	//by default we reset any former value stored in the 'enabled' scalar field
	const ScalarType resetValue = maxSearchSquareDistd <= 0 ? NAN_VALUE : params.maxSearchDist;
	if (params.resetFormerDistances)
	{
		for (unsigned i = 0; i < comparedCloud->size(); ++i)
		{
			comparedCloud->setPointScalarValue(i, resetValue);
		}
	}

	//specific case: a max search distance has been defined and octrees are totally disjoint
	if (maxSearchSquareDistd > 0 && soCode == DISJOINT)
	{
		//nothing to do! (all points are farther than 'maxSearchDist'
		return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
	}

	//if necessary we try to guess the best octree level for distances computation
	if (params.octreeLevel == 0 && referenceOctree) //DGM: referenceOctree can be 0 if the input entities bounding-boxes are disjoint!
	{
		params.octreeLevel = comparedOctree->findBestLevelForComparisonWithOctree(referenceOctree);
	}

	//whether to compute split distances or not
	bool computeSplitDistances = false;
	{
		for (int i = 0; i < 3; ++i)
		{
			if (params.splitDistances[i] && params.splitDistances[i]->currentSize() == comparedCloud->size())
			{
				computeSplitDistances = true;
				params.splitDistances[i]->fill(NAN_VALUE);
			}
		}
	}

	//additional parameters
	void* additionalParameters[] = {	reinterpret_cast<void*>(referenceCloud),
										reinterpret_cast<void*>(referenceOctree),
										reinterpret_cast<void*>(&params),
										reinterpret_cast<void*>(&maxSearchSquareDistd),
										reinterpret_cast<void*>(&computeSplitDistances)
	};

	int result = DISTANCE_COMPUTATION_RESULTS::SUCCESS;

	if (!comparedOctree)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDOCTREE;
	}
	
	result = comparedOctree->executeFunctionForAllCellsAtLevel(params.octreeLevel,
		params.localModel == NO_MODEL ? computeCellHausdorffDistance : computeCellHausdorffDistanceWithLocalModel,
		additionalParameters,
		params.multiThread,
		progressCb,
		"Cloud-Cloud Distance",
		params.maxThreadCount);
	if(result == 0) //executeFunctionForAllCellsAtLevel returns zero if error or canceled
	{
		//something went wrong
		result = DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_FUNCTION_FOR_ALL_CELLS_AT_LEVEL_FAILURE;				
	}
	

	if (comparedOctree && !compOctree)
	{
		delete comparedOctree;
		comparedOctree = nullptr;
	}
	if (referenceOctree && !refOctree)
	{
		delete referenceOctree;
		referenceOctree = nullptr;
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
	if (!comparedCloud || !referenceCloud)
	{
		return EMPTY_CLOUD;
	}
	unsigned nA = comparedCloud->size();
	unsigned nB = referenceCloud->size();

	if (nA == 0 || nB == 0)
	{
		return EMPTY_CLOUD;
	}

	//we compute the bounding box of BOTH clouds
	CCVector3 minsA;
	CCVector3 minsB;
	CCVector3 maxsA;
	CCVector3 maxsB;
	comparedCloud->getBoundingBox(minsA, maxsA);
	referenceCloud->getBoundingBox(minsB, maxsB);

	//we compute the union of both bounding-boxes
	CCVector3 maxD;
	CCVector3 minD;
	{
		for (unsigned char k=0; k<3; k++)
		{
			minD.u[k] = std::min(minsA.u[k], minsB.u[k]);
			maxD.u[k] = std::max(maxsA.u[k], maxsB.u[k]);
		}
	}

	if (maxDist > 0)
	{
		//we reduce the bounding box to the intersection of both bounding-boxes enlarged by 'maxDist'
		for (unsigned char k=0; k<3; k++)
		{
			minD.u[k] = std::max(minD.u[k], std::max(minsA.u[k], minsB.u[k]) - maxDist);
			maxD.u[k] = std::min(maxD.u[k], std::min(maxsA.u[k], maxsB.u[k]) + maxDist);
			if (minD.u[k] > maxD.u[k])
			{
				return DISJOINT;
			}
		}
	}

	CCVector3 minPoints = minD;
	CCVector3 maxPoints = maxD;

	//we make this bounding-box cubical (+1% growth to avoid round-off issues)
	CCMiscTools::MakeMinAndMaxCubical(minD, maxD, 0.01);

	//then we (re)compute octree A if necessary
	bool needToRecalculateOctreeA = true;
	if (comparedOctree && comparedOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeA = false;
		for (unsigned char k=0; k<3; k++)
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

		if (comparedOctree->build(minD, maxD, &minPoints, &maxPoints, progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = nullptr;
			}
			return OUT_OF_MEMORY;
		}
	}

	//and we (re)compute octree B as well if necessary
	bool needToRecalculateOctreeB = true;
	if (referenceOctree && referenceOctree->getNumberOfProjectedPoints() != 0)
	{
		needToRecalculateOctreeB = false;
		for (unsigned char k=0; k<3; k++)
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

		if (referenceOctree->build(minD, maxD, &minPoints, &maxPoints, progressCb) < 1)
		{
			if (octreeACreated)
			{
				delete comparedOctree;
				comparedOctree = nullptr;
			}
			if (octreeBCreated)
			{
				delete referenceOctree;
				referenceOctree = nullptr;
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
	const GenericIndexedCloudPersist* referenceCloud	= reinterpret_cast<GenericIndexedCloudPersist*>(additionalParameters[0]);
	const DgmOctree* referenceOctree					= reinterpret_cast<DgmOctree*>(additionalParameters[1]);
	Cloud2CloudDistanceComputationParams* params		= reinterpret_cast<Cloud2CloudDistanceComputationParams*>(additionalParameters[2]);
	const double* maxSearchSquareDistd					= reinterpret_cast<double*>(additionalParameters[3]);
	bool computeSplitDistances							= *reinterpret_cast<bool*>(additionalParameters[4]);

	//structure for the nearest neighbor search
	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level								= cell.level;
	nNSS.alreadyVisitedNeighbourhoodSize	= 0;
	nNSS.theNearestPointIndex				= 0;
	nNSS.maxSearchSquareDistd				= *maxSearchSquareDistd;

	//we can already compute the position of the 'equivalent' cell in the reference octree
	referenceOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	//and we deduce its center
	referenceOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//for each point of the current cell (compared octree) we look for its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		if (params->CPSet || referenceCloud->testVisibility(nNSS.queryPoint) == POINT_VISIBLE) //to build the closest point set up we must process the point whatever its visibility is!
		{
			double squareDist = referenceOctree->findTheNearestNeighborStartingFromCell(nNSS);
			if (squareDist >= 0)
			{
				ScalarType dist = static_cast<ScalarType>(sqrt(squareDist));
				cell.points->setPointScalarValue(i, dist);

				if (params->CPSet)
				{
					params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i), nNSS.theNearestPointIndex);
				}

				if (computeSplitDistances)
				{
					CCVector3 P;
					referenceCloud->getPoint(nNSS.theNearestPointIndex, P);
					
					unsigned index = cell.points->getPointGlobalIndex(i);
					if (params->splitDistances[0])
						params->splitDistances[0]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.x - P.x));
					if (params->splitDistances[1])
						params->splitDistances[1]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.y - P.y));
					if (params->splitDistances[2])
						params->splitDistances[2]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.z - P.z));
				}
			}
			else
			{
				assert(!params->CPSet);
			}
		}
		else
		{
			cell.points->setPointScalarValue(i, NAN_VALUE);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
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
	GenericIndexedCloudPersist* referenceCloud		= reinterpret_cast<GenericIndexedCloudPersist*>(additionalParameters[0]);
	const DgmOctree* referenceOctree				= reinterpret_cast<DgmOctree*>(additionalParameters[1]);
	Cloud2CloudDistanceComputationParams* params	= reinterpret_cast<Cloud2CloudDistanceComputationParams*>(additionalParameters[2]);
	const double* maxSearchSquareDistd				= reinterpret_cast<double*>(additionalParameters[3]);
	bool computeSplitDistances						= *reinterpret_cast<bool*>(additionalParameters[4]);

	assert(params && params->localModel != NO_MODEL);

	//structure for the nearest neighbor search
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
		nNSS_Model.prepare(static_cast<PointCoordinateType>(params->radiusForLocalModel), cell.parentOctree->getCellSize(cell.level));
	}
	else
	{
		nNSS_Model.minNumberOfNeighbors = params->kNNForLocalModel;
	}

	//already computed models
	std::vector<const LocalModel*> models;

	//for each point of the current cell (compared octree) we look for its nearest neighbour in the reference cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; ++i)
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
				referenceCloud->getPoint(nNSS.theNearestPointIndex, nearestPoint);

				//local model for the 'nearest point'
				const LocalModel* lm = nullptr;

				if (params->reuseExistingLocalModels)
				{
					//we look if the nearest point is close to existing models
					for (std::vector<const LocalModel*>::const_iterator it = models.begin(); it != models.end(); ++it)
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
						Tuple3i cellPos;
						referenceOctree->getTheCellPosWhichIncludesThePoint(&nearestPoint, cellPos, cell.level, inbounds);
						//if the cell is different or the structure has not yet been initialized, we reset it!
						if (	cellPos.x != nNSS_Model.cellPos.x
							||	cellPos.y != nNSS_Model.cellPos.y
							||	cellPos.z != nNSS_Model.cellPos.z)
						{
							nNSS_Model.cellPos = cellPos;
							referenceOctree->computeCellCenter(nNSS_Model.cellPos, nNSS_Model.level, nNSS_Model.cellCenter);
							assert(inbounds);
							nNSS_Model.minimalCellsSetToVisit.clear();
							nNSS_Model.pointsInNeighbourhood.clear();
							nNSS_Model.alreadyVisitedNeighbourhoodSize = inbounds ? 0 : 1;
							//nNSS_Model.theNearestPointIndex = 0;
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
						kNN = std::min(kNN, params->kNNForLocalModel);
					}

					//if there's enough neighbours
					if (kNN >= CC_LOCAL_MODEL_MIN_SIZE[params->localModel])
					{
						DgmOctreeReferenceCloud neighboursCloud(&nNSS_Model.pointsInNeighbourhood,kNN);
						Neighbourhood Z(&neighboursCloud);

						//Neighbours are sorted, so the farthest is at the end. It also gives us
						//an approximation of the model 'size'
						const double& maxSquareDist = nNSS_Model.pointsInNeighbourhood[kNN-1].squareDistd;
						if (maxSquareDist > 0) //DGM: with duplicate points, all neighbors can be at the same place :(
						{
							lm = LocalModel::New(params->localModel, Z, nearestPoint, static_cast<PointCoordinateType>(maxSquareDist));
							if (lm && params->reuseExistingLocalModels)
							{
								//we add the model to the 'existing models' list
								try
								{
									models.push_back(lm);
								}
								catch (const std::bad_alloc&)
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
					CCVector3 nearestModelPoint;
					ScalarType distToModel = lm->computeDistanceFromModelToPoint(&nNSS.queryPoint, computeSplitDistances ? &nearestModelPoint : nullptr);

					//we take the best estimation between the nearest neighbor and the model!
					//this way we only reduce any potential noise (that would be due to sampling)
					//instead of 'adding' noise if the model is badly shaped
					if (distToNearestPoint <= distToModel)
					{
						distPt = distToNearestPoint;
					}
					else
					{
						distPt = distToModel;
						nearestPoint = nearestModelPoint;
					}

					if (!params->reuseExistingLocalModels)
					{
						//we don't need the local model anymore!
						delete lm;
						lm = nullptr;
					}

					if (computeSplitDistances)
					{
						unsigned index = cell.points->getPointGlobalIndex(i);
						if (params->splitDistances[0])
							params->splitDistances[0]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.x - nearestPoint.x));
						if (params->splitDistances[1])
							params->splitDistances[1]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.y - nearestPoint.y));
						if (params->splitDistances[2])
							params->splitDistances[2]->setValue(index, static_cast<ScalarType>(nNSS.queryPoint.z - nearestPoint.z));
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
			{
				params->CPSet->setPointIndex(cell.points->getPointGlobalIndex(i), nNSS.theNearestPointIndex);
			}
		}
	
		cell.points->setPointScalarValue(i, distPt);

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	//clear all models for this cell
	while (!models.empty())
	{
		delete models.back();
		models.pop_back();
	}

	return true;
}

//Internal structure used by DistanceComputationTools::computeCloud2MeshDistance
struct CellToTest
{
	//Warning: put the non aligned members (< 4 bytes) at the end to avoid too much alignment padding!

	//! Cell position
	Tuple3i pos;				//12 bytes
	//! Cell size
	int cellSize;				// 4 bytes
	//! Subdivision level
	unsigned char level;		// 1 byte (+ 3 for alignment)

	//Total						//20 bytes
};

int DistanceComputationTools::intersectMeshWithOctree(	OctreeAndMeshIntersection* intersection,
														unsigned char octreeLevel,
														GenericProgressCallback* progressCb/*=0*/)
{
	if (!intersection)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_OCTREE_AND_MESH_INTERSECTION;
	}

	DgmOctree* octree = intersection->octree;
	GenericIndexedMesh* mesh = intersection->mesh;
	if (!octree)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_OCTREE;
	}

	if (!mesh)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEMESH;
	}

	//cell dimension
	PointCoordinateType cellLength = octree->getCellSize(octreeLevel);
	CCVector3 halfCellDimensions(cellLength / 2, cellLength / 2, cellLength / 2);
	
	std::vector<CellToTest> cellsToTest(1); //initial size must be > 0
	unsigned cellsToTestCount = 0;

	//get octree box
	const CCVector3& minBB = octree->getOctreeMins();
	//and the number of triangles
	unsigned numberOfTriangles = mesh->size();

	//for progress notification
	NormalizedProgress nProgress(progressCb,numberOfTriangles);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[64];
			sprintf(buffer, "Triangles: %u", numberOfTriangles);
			progressCb->setInfo(buffer);
			progressCb->setMethodTitle("Intersect Grid/Mesh");
		}
		progressCb->update(0);
		progressCb->start();
	}

	//For each triangle: look for intersecting cells
	mesh->placeIteratorAtBeginning();
	int result = DISTANCE_COMPUTATION_RESULTS::SUCCESS;
	for (unsigned n=0; n<numberOfTriangles; ++n)
	{
		//get the positions (in the grid) of each vertex 
		const GenericTriangle* T = mesh->_getNextTriangle();
		const CCVector3* triPoints[3] = {	T->_getA(),
											T->_getB(),
											T->_getC() };

		CCVector3 AB = (*triPoints[1]) - (*triPoints[0]);
		CCVector3 BC = (*triPoints[2]) - (*triPoints[1]);
		CCVector3 CA = (*triPoints[0]) - (*triPoints[2]);

		//be sure that the triangle is not degenerate!!!
		if (AB.norm2() > ZERO_TOLERANCE &&
			BC.norm2() > ZERO_TOLERANCE &&
			CA.norm2() > ZERO_TOLERANCE)
		{
			Tuple3i cellPos[3];
			octree->getTheCellPosWhichIncludesThePoint(triPoints[0], cellPos[0], octreeLevel);
			octree->getTheCellPosWhichIncludesThePoint(triPoints[1], cellPos[1], octreeLevel);
			octree->getTheCellPosWhichIncludesThePoint(triPoints[2], cellPos[2], octreeLevel);

			//compute the triangle bounding-box
			Tuple3i minPos;
			Tuple3i maxPos;
			for (int k=0; k<3; k++)
			{
				minPos.u[k] = std::min( cellPos[0].u[k], std::min( cellPos[1].u[k],cellPos[2].u[k] ));
				maxPos.u[k] = std::max( cellPos[0].u[k], std::max( cellPos[1].u[k],cellPos[2].u[k] ));
			}

			//first cell
			assert(cellsToTest.capacity() != 0);
			cellsToTestCount = 1;
			CellToTest* _currentCell = &cellsToTest[0/*cellsToTestCount-1*/];

			_currentCell->pos = minPos;
			CCVector3 distanceToOctreeMinBorder = minBB - (*triPoints[0]);

			//compute the triangle normal
			CCVector3 N = AB.cross(BC);

			//max distance (in terms of cell) between the vertices
			int maxSize = 0;
			{
				Tuple3i delta = maxPos - minPos + Tuple3i(1,1,1);
				maxSize = std::max(delta.x, delta.y);
				maxSize = std::max(maxSize, delta.z);
			}

			//we deduce the smallest bounding 'octree' cell
			//(not a real octree cell in fact as its starting position is anywhere in the grid
			//and it can even 'outbounds' the grid, i.e. currentCell.pos.u[k]+currentCell.cellSize > octreeLength)
			static const double LOG_2 = log(2.0);
			_currentCell->level = octreeLevel-(maxSize > 1 ? static_cast<unsigned char>(ceil(log(static_cast<double>(maxSize))/LOG_2)) : 0);
			_currentCell->cellSize = (1 << (octreeLevel-_currentCell->level));

			//now we can (recursively) find the intersecting cells
			while (cellsToTestCount != 0)
			{
				_currentCell = &cellsToTest[--cellsToTestCount];

				//new cells may be written over the actual one
				//so we need to remember its position!
				Tuple3i currentCellPos = _currentCell->pos;

				//if we have reached the maximal subdivision level
				if (_currentCell->level == octreeLevel)
				{
					//compute the (absolute) cell center
					octree->computeCellCenter(currentCellPos,octreeLevel,AB);

					//check that the triangle do intersects the cell (box)
					if (CCMiscTools::TriBoxOverlap(AB, halfCellDimensions, triPoints))
					{
						if ((currentCellPos.x >= intersection->minFillIndexes.x && currentCellPos.x <= intersection->maxFillIndexes.x) &&
							(currentCellPos.y >= intersection->minFillIndexes.y && currentCellPos.y <= intersection->maxFillIndexes.y) &&
							(currentCellPos.z >= intersection->minFillIndexes.z && currentCellPos.z <= intersection->maxFillIndexes.z) )
						{
							Tuple3i cellPos = currentCellPos - intersection->minFillIndexes;

							if (intersection->perCellTriangleList.isInitialized())
							{
								TriangleList*& triList = intersection->perCellTriangleList.getValue(cellPos);
								if (!triList)
								{
									triList = new TriangleList();
									//triList->cellCode = DgmOctree::GenerateTruncatedCellCode(currentCellPos, octreeLevel);
								}

								//add the triangle to the current 'intersecting triangles' list
								triList->push(n);
							}

							if (intersection->distanceTransform)
							{
								intersection->distanceTransform->setValue(cellPos, 1);
							}
						}
					}
				}
				else
				{
					int halfCellSize = (_currentCell->cellSize >> 1);

					//compute the position of each cell 'neighbors' relatively to the triangle (3*3*3 = 27, including the cell itself)
					char pointsPosition[27];
					{
						char* _pointsPosition = pointsPosition;
						for (int i=0; i<3; ++i)
						{
							AB.x = distanceToOctreeMinBorder.x + static_cast<PointCoordinateType>(currentCellPos.x + i*halfCellSize) * cellLength;
							for (int j=0; j<3; ++j)
							{
								AB.y = distanceToOctreeMinBorder.y + static_cast<PointCoordinateType>(currentCellPos.y + j*halfCellSize) * cellLength;
								for (int k=0; k<3; ++k)
								{
									AB.z = distanceToOctreeMinBorder.z + static_cast<PointCoordinateType>(currentCellPos.z + k*halfCellSize) * cellLength;

									//determine on which side the triangle is
									*_pointsPosition++/*pointsPosition[i*9+j*3+k]*/ = (AB.dot(N) < 0 ? -1 : 1);
								}
							}
						}
					}

					//if necessary we enlarge the queue
					if (cellsToTestCount + 27 > cellsToTest.capacity())
					{
						try
						{
							cellsToTest.resize(std::max(cellsToTest.capacity()+27, 2*cellsToTest.capacity()));
						}
						catch (const std::bad_alloc&)
						{
							//out of memory
							return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
						}
					}

					//the first new cell will be written over the actual one
					CellToTest* _newCell = &cellsToTest[cellsToTestCount];
					_newCell->level++;
					_newCell->cellSize = halfCellSize;

					//we look at the position of the 8 sub-cubes relatively to the triangle
					for (int i=0; i<2; ++i)
					{
						_newCell->pos.x = currentCellPos.x + i*halfCellSize;
						//quick test to determine if the cube is potentially intersecting the triangle's bbox
						if (	static_cast<int>(_newCell->pos.x) + halfCellSize >= minPos.x
							&&	static_cast<int>(_newCell->pos.x)				 <= maxPos.x )
						{
							for (int j=0; j<2; ++j)
							{
								_newCell->pos.y = currentCellPos.y + j*halfCellSize;
								if (	static_cast<int>(_newCell->pos.y) + halfCellSize >= minPos.y
									&&	static_cast<int>(_newCell->pos.y)				 <= maxPos.y )
								{
									for (int k=0; k<2; ++k)
									{
										_newCell->pos.z = currentCellPos.z + k*halfCellSize;
										if (	static_cast<int>(_newCell->pos.z) + halfCellSize >= minPos.z
											&&	static_cast<int>(_newCell->pos.z)				 <= maxPos.z )
										{
											const char* _pointsPosition = pointsPosition + (i*9+j*3+k);
											char sum =	  _pointsPosition[0]  + _pointsPosition[1]	+ _pointsPosition[3]
														+ _pointsPosition[4]  + _pointsPosition[9]	+ _pointsPosition[10]
														+ _pointsPosition[12] + _pointsPosition[13];

											//if all the sub-cube vertices are not on the same side, then the triangle may intersect the cell
											if (std::abs(sum) < 8)
											{
												//we make newCell point on next cell in array (we copy current info by the way)
												cellsToTest[++cellsToTestCount] = *_newCell;
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

		if (progressCb && !nProgress.oneStep())
		{
			//cancel by user
			result = DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER;
			break;
		}
	}

	return result;
}

//! Method used by computeCloud2MeshDistanceWithOctree
void ComparePointsAndTriangles( ReferenceCloud& Yk,
								unsigned& remainingPoints,
								CCLib::GenericIndexedMesh* mesh,
								std::vector<unsigned>& trianglesToTest,
								std::size_t& trianglesToTestCount,
								std::vector<ScalarType>& minDists,
								ScalarType maxRadius,
								CCLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams& params)
{
	assert(mesh);
	assert(remainingPoints <= Yk.size());
	assert(trianglesToTestCount <= trianglesToTest.size());

	bool firstComparisonDone = (trianglesToTestCount != 0);

	CCVector3 nearestPoint;
	CCVector3* _nearestPoint = params.CPSet ? &nearestPoint : nullptr;

	//for each triangle
	while (trianglesToTestCount != 0)
	{
		//we query the vertex coordinates
		CCLib::SimpleTriangle tri;
		mesh->getTriangleVertices(trianglesToTest[--trianglesToTestCount], tri.A, tri.B, tri.C);

		//for each point inside the current cell
		if (params.signedDistances)
		{
			//we have to use absolute distances
			for (unsigned j=0; j<remainingPoints; ++j)
			{
				//compute the distance to the triangle
				ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getPoint(j), &tri, true, _nearestPoint);
				//keep it if it's smaller
				ScalarType min_d = Yk.getPointScalarValue(j);
				if (!ScalarField::ValidValue(min_d) || min_d*min_d > dPTri*dPTri)
				{
					Yk.setPointScalarValue(j, params.flipNormals ? -dPTri : dPTri);
					if (params.CPSet)
					{
						//Closest Point Set: save the nearest point as well
						assert(_nearestPoint);
						*const_cast<CCVector3*>(params.CPSet->getPoint(Yk.getPointGlobalIndex(j))) = *_nearestPoint;
					}
				}
			}
		}
		else //squared distances
		{
			for (unsigned j = 0; j<remainingPoints; ++j)
			{
				//compute the (SQUARED) distance to the triangle
				ScalarType dPTri = DistanceComputationTools::computePoint2TriangleDistance(Yk.getPoint(j), &tri, false, _nearestPoint);
				//keep it if it's smaller
				ScalarType min_d = Yk.getPointScalarValue(j);
				if (!ScalarField::ValidValue(min_d) || dPTri < min_d)
				{
					Yk.setPointScalarValue(j, dPTri);
					if (params.CPSet)
					{
						//Closest Point Set: save the nearest point as well
						assert(_nearestPoint);
						*const_cast<CCVector3*>(params.CPSet->getPoint(Yk.getPointGlobalIndex(j))) = *_nearestPoint;
					}
				}
			}
		}
	}

	//we can 'remove' all the eligible points at the current neighborhood radius
	if (firstComparisonDone)
	{
		Yk.placeIteratorAtBeginning();
		for (unsigned j=0; j<remainingPoints; ++j)
		{
			//eligibility distance
			ScalarType eligibleDist = minDists[j] + maxRadius;
			ScalarType dPTri = Yk.getCurrentPointScalarValue();
			if (params.signedDistances)
			{
				//need to get the square distance in all cases
				dPTri *= dPTri;
			}
			if (dPTri <= eligibleDist*eligibleDist)
			{
				//remove this point
				Yk.removeCurrentPointGlobalIndex();
				//and do the same for the 'minDists' array! (see ReferenceCloud::removeCurrentPointGlobalIndex)
				assert(remainingPoints != 0);
				minDists[j] = minDists[--remainingPoints];
				//minDists.pop_back();
				--j;
			}
			else
			{
				Yk.forwardIterator();
			}
		}
	}
}


int ComputeMaxNeighborhoodLength(ScalarType maxSearchDist, PointCoordinateType cellSize)
{
	return static_cast<int>(ceil(maxSearchDist / cellSize + static_cast<ScalarType>((sqrt(2.0) - 1.0) / 2)));
}

#ifdef ENABLE_CLOUD2MESH_DIST_MT

#include <QtCore>
#include <QApplication>
#include <QtConcurrentMap>

/*** MULTI THREADING WRAPPER ***/
static DgmOctree* s_octree_MT = nullptr;
static NormalizedProgress* s_normProgressCb_MT = nullptr;
static OctreeAndMeshIntersection* s_intersection_MT = nullptr;
static bool s_cellFunc_MT_success = true;
static int s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::SUCCESS;
static CCLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams s_params_MT;

//'processTriangles' mechanism (based on bit mask)
static std::vector<std::vector<bool>*> s_bitArrayPool_MT;
static bool s_useBitArrays_MT = true;
static QMutex s_currentBitMaskMutex;

void cloudMeshDistCellFunc_MT(const DgmOctree::IndexAndCode& desc)
{
	if (!s_cellFunc_MT_success)
	{
		//skip this cell if the process is aborted / has failed
		return;
	}

	if (s_normProgressCb_MT && !s_normProgressCb_MT->oneStep())
	{
		s_cellFunc_MT_success = false;
		s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER;
		return;
	}

	ReferenceCloud Yk(s_octree_MT->associatedCloud());
	s_octree_MT->getPointsInCellByCellIndex(&Yk, desc.theIndex, s_params_MT.octreeLevel);

	//min distance array
	unsigned remainingPoints = Yk.size();

	std::vector<ScalarType> minDists;
	try
	{
		minDists.resize(remainingPoints);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		s_cellFunc_MT_success = false;
		s_cellFunc_MT_results = DistanceComputationTools::DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		return;
	}

	//get cell pos
	Tuple3i startPos;
	s_octree_MT->getCellPos(desc.theCode, s_params_MT.octreeLevel, startPos, true);

	//get the distance to the nearest and farthest boundaries
	int maxDistToBoundaries = 0;
	Tuple3i distToLowerBorder = startPos - s_intersection_MT->minFillIndexes;
	Tuple3i distToUpperBorder = s_intersection_MT->maxFillIndexes - startPos;
	for (unsigned k = 0; k<3; ++k)
	{
		maxDistToBoundaries = std::max(maxDistToBoundaries, distToLowerBorder.u[k]);
		maxDistToBoundaries = std::max(maxDistToBoundaries, distToUpperBorder.u[k]);
	}
	int maxIntDist = maxDistToBoundaries;

	if (s_params_MT.maxSearchDist > 0)
	{
		//no need to look farther than 'maxNeighbourhoodLength'
		int maxNeighbourhoodLength = ComputeMaxNeighborhoodLength(s_params_MT.maxSearchDist, s_octree_MT->getCellSize(s_params_MT.octreeLevel));
		if (maxNeighbourhoodLength < maxIntDist)
			maxIntDist = maxNeighbourhoodLength;

		ScalarType maxDistance = s_params_MT.maxSearchDist;
		if (!s_params_MT.signedDistances)
		{
			//we compute squared distances when not in 'signed' mode!
			maxDistance = s_params_MT.maxSearchDist*s_params_MT.maxSearchDist;
		}

		for (unsigned j = 0; j < remainingPoints; ++j)
			Yk.setPointScalarValue(j, maxDistance);
	}

	//determine the cell center
	CCVector3 cellCenter;
	s_octree_MT->computeCellCenter(startPos, s_params_MT.octreeLevel, cellCenter);

	//express 'startPos' relatively to the grid borders
	startPos -= s_intersection_MT->minFillIndexes;

	//octree cell size
	const PointCoordinateType& cellLength = s_octree_MT->getCellSize(s_params_MT.octreeLevel);

	//useful variables
	std::vector<unsigned> trianglesToTest;
	std::size_t trianglesToTestCount = 0;
	std::size_t trianglesToTestCapacity = 0;

	//bit mask for efficient comparisons
	std::vector<bool>* bitArray = nullptr;
	if (s_useBitArrays_MT)
	{
		const unsigned numTri = s_intersection_MT->mesh->size();
		s_currentBitMaskMutex.lock();
		if (s_bitArrayPool_MT.empty())
		{
			bitArray = new std::vector<bool>(numTri);
		}
		else
		{
			bitArray = s_bitArrayPool_MT.back();
			s_bitArrayPool_MT.pop_back();
		}
		s_currentBitMaskMutex.unlock();
		bitArray->assign(numTri, false);
	}

	//for each point, we pre-compute its distance to the nearest cell border
	//(will be handy later)
	Yk.placeIteratorAtBeginning();
	for (unsigned j = 0; j<remainingPoints; ++j)
	{
		//coordinates of the current point
		const CCVector3 *tempPt = Yk.getCurrentPointCoordinates();
		//distance du bord le plus proche = taille de la cellule - distance la plus grande par rapport au centre de la cellule
		minDists[j] = DgmOctree::ComputeMinDistanceToCellBorder(*tempPt, cellLength, cellCenter);
		Yk.forwardIterator();
	}

	//let's find the nearest triangles for each point in the neighborhood 'Yk'
	ScalarType maxRadius = 0;
	for (int dist = 0; remainingPoints != 0 && dist <= maxIntDist; ++dist, maxRadius += static_cast<ScalarType>(cellLength))
	{
		//test the neighbor cells at distance = 'dist'
		//a,b,c,d,e,f are the extents of this neighborhood
		//for the 6 main directions -X,+X,-Y,+Y,-Z,+Z
		int a = std::min(dist, distToLowerBorder.x);
		int b = std::min(dist, distToUpperBorder.x);
		int c = std::min(dist, distToLowerBorder.y);
		int d = std::min(dist, distToUpperBorder.y);
		int e = std::min(dist, distToLowerBorder.z);
		int f = std::min(dist, distToUpperBorder.z);

		for (int i = -a; i <= b; ++i)
		{
			bool imax = (std::abs(i) == dist);
			Tuple3i cellPos(startPos.x + i, 0, 0);

			for (int j = -c; j <= d; j++)
			{
				cellPos.y = startPos.y + j;

				//if i or j is 'maximal'
				if (imax || std::abs(j) == dist)
				{
					//we must be on the border of the neighborhood
					for (int k = -e; k <= f; k++)
					{
						//are there any triangles near this cell?
						cellPos.z = startPos.z+k;
						TriangleList* triList = s_intersection_MT->perCellTriangleList.getValue(cellPos);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}
				}
				else //we must go the cube border
				{
					if (e == dist) //'negative' side
					{
						//are there any triangles near this cell?
						cellPos.z = startPos.z - e;
						TriangleList* triList = s_intersection_MT->perCellTriangleList.getValue(cellPos);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}

					if (f == dist && dist>0) //'positive' side
					{
						//are there any triangles near this cell?
						cellPos.z = startPos.z + f;
						TriangleList* triList = s_intersection_MT->perCellTriangleList.getValue(cellPos);
						if (triList)
						{
							if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
							{
								trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
								trianglesToTest.resize(trianglesToTestCapacity);
							}
							//let's test all the triangles that intersect this cell
							for (unsigned p = 0; p<triList->indexes.size(); ++p)
							{
								if (bitArray)
								{
									const unsigned& indexTri = triList->indexes[p];
									//if the triangles has not been processed yet
									if (!(*bitArray)[indexTri])
									{
										trianglesToTest[trianglesToTestCount++] = indexTri;
										(*bitArray)[indexTri] = true;
									}
								}
								else
								{
									trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
								}
							}
						}
					}
				}
			}
		}

		ComparePointsAndTriangles(Yk, remainingPoints, s_intersection_MT->mesh, trianglesToTest, trianglesToTestCount, minDists, maxRadius, s_params_MT);
	}

	//Save the bit mask
	if (bitArray)
	{
		s_currentBitMaskMutex.lock();
		s_bitArrayPool_MT.push_back(bitArray);
		s_currentBitMaskMutex.unlock();
	}
}

#endif

int DistanceComputationTools::computeCloud2MeshDistanceWithOctree(	OctreeAndMeshIntersection* intersection,
																	Cloud2MeshDistanceComputationParams& params,
																	GenericProgressCallback* progressCb/*=0*/)
{
	assert(intersection);
	assert(!params.signedDistances || !intersection->distanceTransform); //signed distances are not compatible with Distance Transform acceleration
	assert(!params.multiThread || params.maxSearchDist <= 0); //maxSearchDist is not compatible with parallel processing
	if (!intersection)
	{
		//invalid input
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_OCTREE_AND_MESH_INTERSECTION;
	}
	DgmOctree* octree = intersection->octree;
	if (!octree)
	{
		//invalid input
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_OCTREE;
	}

	//Closest Point Set
	if (params.CPSet)
	{
		assert(params.maxSearchDist <= 0);
		assert(params.useDistanceMap == false);

		//reserve memory for the Closest Point Set
		if (!params.CPSet->resize(octree->associatedCloud()->size()))
		{
			//not enough memory
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}
	}

#ifdef ENABLE_CLOUD2MESH_DIST_MT
	if (!params.multiThread)
#endif
	{
		GenericIndexedMesh* mesh = intersection->mesh;

		//dimension of an octree cell
		PointCoordinateType cellLength = octree->getCellSize(params.octreeLevel);

		//get the cell indexes at level "octreeLevel"
		DgmOctree::cellsContainer cellCodesAndIndexes;
		if (!octree->getCellCodesAndIndexes(params.octreeLevel, cellCodesAndIndexes, true))
		{
			//not enough memory
			return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
		}

		unsigned numberOfCells = static_cast<unsigned>(cellCodesAndIndexes.size());
		bool boundedSearch = (params.maxSearchDist > 0);

		DgmOctree::cellsContainer::const_iterator pCodeAndIndex = cellCodesAndIndexes.begin();
		ReferenceCloud Yk(octree->associatedCloud());

		//if we only need approximate distances
		if (intersection->distanceTransform)
		{
			//for each cell
			for (unsigned i = 0; i < numberOfCells; ++i, ++pCodeAndIndex)
			{
				octree->getPointsInCellByCellIndex(&Yk, pCodeAndIndex->theIndex, params.octreeLevel);

				//get the cell pos
				Tuple3i cellPos;
				octree->getCellPos(pCodeAndIndex->theCode, params.octreeLevel, cellPos, true);
				cellPos -= intersection->minFillIndexes;

				//get the Distance Transform distance
				unsigned squareDist = intersection->distanceTransform->getValue(cellPos);

				//assign the distance to all points inside this cell
				ScalarType maxRadius = sqrt(static_cast<ScalarType>(squareDist)) * cellLength;

				if (boundedSearch && maxRadius > params.maxSearchDist)
				{
					maxRadius = params.maxSearchDist;
				}

				unsigned count = Yk.size();
				for (unsigned j = 0; j < count; ++j)
				{
					Yk.setPointScalarValue(j, maxRadius);
				}

				//Yk.clear(); //useless
			}

			return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
		}

		//otherwise we have to compute the distance from each point to its nearest triangle

		//Progress callback
		NormalizedProgress nProgress(progressCb, numberOfCells);
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Cells: %u", numberOfCells);
				progressCb->setInfo(buffer);
				progressCb->setMethodTitle(params.signedDistances ? "Compute signed distances" : "Compute distances");
			}
			progressCb->update(0);
			progressCb->start();
		}

		//variables
		std::vector<unsigned> trianglesToTest;
		std::size_t trianglesToTestCount = 0;
		std::size_t trianglesToTestCapacity = 0;
		unsigned numberOfTriangles = mesh->size();

		//acceleration structure
		std::vector<unsigned> processTriangles;
		try
		{
			processTriangles.resize(numberOfTriangles, 0);
		}
		catch (const std::bad_alloc&)
		{
			//otherwise, no big deal, we can do without it!
		}

		//min distance array ('persistent' version to save some memory)
		std::vector<ScalarType> minDists;

		//maximal neighbors search distance (if maxSearchDist is defined)
		int maxNeighbourhoodLength = 0; 
		if (boundedSearch)
		{
			maxNeighbourhoodLength = ComputeMaxNeighborhoodLength(params.maxSearchDist, cellLength);
		}

		//for each cell
		for (unsigned cellIndex = 1; cellIndex <= numberOfCells; ++cellIndex, ++pCodeAndIndex) //cellIndex = unique ID for the current cell
		{
			if (!octree->getPointsInCellByCellIndex(&Yk, pCodeAndIndex->theIndex, params.octreeLevel))
			{
				return DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_GET_POINTS_IN_CELL_BY_INDEX_FAILURE;
			}

			//get cell pos
			Tuple3i startPos;
			octree->getCellPos(pCodeAndIndex->theCode, params.octreeLevel, startPos, true);

			//get the distance to the nearest and farthest boundaries
			int maxDistToBoundaries = 0;
			Tuple3i distToLowerBorder = startPos - intersection->minFillIndexes;
			Tuple3i distToUpperBorder = intersection->maxFillIndexes - startPos;
			for (unsigned char k = 0; k < 3; ++k)
			{
				maxDistToBoundaries = std::max(maxDistToBoundaries, distToLowerBorder.u[k]);
				maxDistToBoundaries = std::max(maxDistToBoundaries, distToUpperBorder.u[k]);
			}
			int maxIntDist = maxDistToBoundaries;

			//determine the cell center
			CCVector3 cellCenter;
			octree->computeCellCenter(startPos, params.octreeLevel, cellCenter);

			//express 'startPos' relatively to the grid borders
			startPos -= intersection->minFillIndexes;

			//minDists.clear(); //not necessary 
			unsigned remainingPoints = Yk.size();
			if (minDists.size() < remainingPoints)
			{
				try
				{
					minDists.resize(remainingPoints);
				}
				catch (const std::bad_alloc&) //out of memory
				{
					//not enough memory
					return DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY;
				}
			}

			//for each point, we pre-compute its distance to the nearest cell border
			//(will be handy later)
			for (unsigned j = 0; j < remainingPoints; ++j)
			{
				const CCVector3 *tempPt = Yk.getPointPersistentPtr(j);
				minDists[j] = static_cast<ScalarType>(DgmOctree::ComputeMinDistanceToCellBorder(*tempPt, cellLength, cellCenter));
			}

			if (boundedSearch)
			{
				//no need to look farther than 'maxNeighbourhoodLength'
				if (maxNeighbourhoodLength < maxIntDist)
					maxIntDist = maxNeighbourhoodLength;

				//we compute squared distances when not in 'signed' mode!
				ScalarType maxDistance = params.maxSearchDist;
				if (!params.signedDistances)
				{
					//we compute squared distances when not in 'signed' mode!
					maxDistance = params.maxSearchDist*params.maxSearchDist;
				}
				
				for (unsigned j = 0; j < remainingPoints; ++j)
					Yk.setPointScalarValue(j, maxDistance);
			}

			//let's find the nearest triangles for each point in the neighborhood 'Yk'
			ScalarType maxRadius = 0;
			for (int dist = 0; dist <= maxIntDist && remainingPoints != 0; ++dist, maxRadius += static_cast<ScalarType>(cellLength))
			{
				//test the neighbor cells at distance = 'dist'
				//a,b,c,d,e,f are the extents of this neighborhood
				//for the 6 main directions -X,+X,-Y,+Y,-Z,+Z
				int a = std::min(dist, distToLowerBorder.x);
				int b = std::min(dist, distToUpperBorder.x);
				int c = std::min(dist, distToLowerBorder.y);
				int d = std::min(dist, distToUpperBorder.y);
				int e = std::min(dist, distToLowerBorder.z);
				int f = std::min(dist, distToUpperBorder.z);

				for (int i = -a; i <= b; i++)
				{
					bool imax = (std::abs(i) == dist);
					Tuple3i cellPos(startPos.x + i, 0, 0);

					for (int j = -c; j <= d; j++)
					{
						cellPos.y = startPos.y + j;

						//if i or j is 'maximal'
						if (imax || std::abs(j) == dist)
						{
							//we must be on the border of the neighborhood
							for (int k = -e; k <= f; k++)
							{
								//are there any triangles near this cell?
								cellPos.z = startPos.z+k;
								TriangleList* triList = intersection->perCellTriangleList.getValue(cellPos);
								if (triList)
								{
									if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
									{
										trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2*trianglesToTestCount);
										trianglesToTest.resize(trianglesToTestCapacity);
									}
									//let's test all the triangles that intersect this cell
									for (std::size_t p = 0; p < triList->indexes.size(); ++p)
									{
										if (!processTriangles.empty())
										{
											unsigned indexTri = triList->indexes[p];
											//if the triangles has not been processed yet
											if (processTriangles[indexTri] != cellIndex)
											{
												trianglesToTest[trianglesToTestCount++] = indexTri;
												processTriangles[indexTri] = cellIndex;
											}
										}
										else
										{
											trianglesToTest[trianglesToTestCount++] = triList->indexes[p];
										}
									}
								}
							}
						}
						else //we must go the cube border
						{
							if (e == dist) //'negative' side
							{
								//are there any triangles near this cell?
								cellPos.z = startPos.z - e;
								TriangleList* triList = intersection->perCellTriangleList.getValue(cellPos);
								if (triList)
								{
									if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
									{
										trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
										trianglesToTest.resize(trianglesToTestCapacity);
									}
									//let's test all the triangles that intersect this cell
									for (unsigned int triIndex : triList->indexes)
									{
										if (!processTriangles.empty())
										{
											//if the triangles has not been processed yet
											if (processTriangles[triIndex] != cellIndex)
											{
												trianglesToTest[trianglesToTestCount++] = triIndex;
												processTriangles[triIndex] = cellIndex;
											}
										}
										else
										{
											trianglesToTest[trianglesToTestCount++] = triIndex;
										}
									}
								}
							}

							if (f == dist && dist > 0) //'positive' side
							{
								//are there any triangles near this cell?
								cellPos.z = startPos.z + f;
								TriangleList* triList = intersection->perCellTriangleList.getValue(cellPos);
								if (triList)
								{
									if (trianglesToTestCount + triList->indexes.size() > trianglesToTestCapacity)
									{
										trianglesToTestCapacity = std::max(trianglesToTestCount + triList->indexes.size(), 2 * trianglesToTestCount);
										trianglesToTest.resize(trianglesToTestCapacity);
									}
									//let's test all the triangles that intersect this cell
									for (unsigned int triIndex : triList->indexes)
									{
										if (!processTriangles.empty())
										{
											//if the triangles has not been processed yet
											if (processTriangles[triIndex] != cellIndex)
											{
												trianglesToTest[trianglesToTestCount++] = triIndex;
												processTriangles[triIndex] = cellIndex;
											}
										}
										else
										{
											trianglesToTest[trianglesToTestCount++] = triIndex;
										}
									}
								}
							}
						}
					}
				}

				ComparePointsAndTriangles(Yk, remainingPoints, mesh, trianglesToTest, trianglesToTestCount, minDists, maxRadius, params);
			}

			//Yk.clear(); //not necessary

			if (progressCb && !nProgress.oneStep())
			{
				//process cancelled by the user
				return DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER;
			}
		}

		return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
	}
#ifdef ENABLE_CLOUD2MESH_DIST_MT
	else
	{
		//extraction des indexes et codes des cellules du niveau "octreeLevel"
		DgmOctree::cellsContainer cellsDescs;
		if (!octree->getCellCodesAndIndexes(params.octreeLevel, cellsDescs, true))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_CODES_AND_INDEXES_FAILURE;
		}

		unsigned numberOfCells = static_cast<unsigned>(cellsDescs.size());

		//Progress callback
		NormalizedProgress nProgress(progressCb,numberOfCells);
		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Cells: %u", numberOfCells);
				progressCb->setInfo(buffer);
				progressCb->setMethodTitle("Compute signed distances");
			}
			progressCb->update(0);
			progressCb->start();
		}

		s_octree_MT = octree;
		s_normProgressCb_MT = &nProgress;
		s_cellFunc_MT_success = true;
		s_cellFunc_MT_results = DISTANCE_COMPUTATION_RESULTS::SUCCESS;
		s_params_MT = params;
		s_intersection_MT = intersection;
		//acceleration structure
		s_useBitArrays_MT = true;

		//Single thread emulation
		//for (unsigned i=0; i<numberOfCells; ++i)
		//	cloudMeshDistCellFunc_MT(cellsDescs[i]);

		int maxThreadCount = params.maxThreadCount;
		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(cellsDescs, cloudMeshDistCellFunc_MT);

		s_octree_MT = nullptr;
		s_normProgressCb_MT = nullptr;
		s_intersection_MT = nullptr;

		//clean acceleration structure
		while (!s_bitArrayPool_MT.empty())
		{
			delete s_bitArrayPool_MT.back();
			s_bitArrayPool_MT.pop_back();
		}
		if (!s_cellFunc_MT_success && s_cellFunc_MT_results == DISTANCE_COMPUTATION_RESULTS::SUCCESS)
		{
			s_cellFunc_MT_results = DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_CLOUD_MESH_DIST_CELL_FUNC_MT_FAILURE;
		}
		return (s_cellFunc_MT_success ? DISTANCE_COMPUTATION_RESULTS::SUCCESS : s_cellFunc_MT_results);
	}
#endif
}

//convert all 'distances' (squared in fact) to their square root
inline void applySqrtToPointDist(const CCVector3 &aPoint, ScalarType& aScalarValue)
{
	if (ScalarField::ValidValue(aScalarValue))
		aScalarValue = sqrt(aScalarValue);
}

int DistanceComputationTools::computeCloud2MeshDistance(	GenericIndexedCloudPersist* pointCloud,
															GenericIndexedMesh* mesh,
															Cloud2MeshDistanceComputationParams& params,
															GenericProgressCallback* progressCb/*=0*/,
															DgmOctree* cloudOctree/*=0*/)
{
	//check the input
	if (!pointCloud)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (pointCloud->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (!mesh)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCEMESH;
	}

	if (mesh->size() == 0)
	{
		assert(false);
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCEMESH;
	}


	if (params.signedDistances)
	{
		//signed distances are incompatible with approximate distances (with Distance Transform)
		params.useDistanceMap = false;
	}
	if (params.CPSet)
	{
		//Closest Point Set determination is incompatible with distance map approximation and max search distance
		params.useDistanceMap = false;
		params.maxSearchDist = 0;
	}

	//compute the (cubical) bounding box that contains both the cloud and the mesh BBs
	CCVector3 cloudMinBB;
	CCVector3 cloudMaxBB;
	CCVector3 meshMinBB;
	CCVector3 meshMaxBB;
	CCVector3 minBB;
	CCVector3 maxBB;
	CCVector3 minCubifiedBB;
	CCVector3 maxCubifiedBB;
	{
		pointCloud->getBoundingBox(cloudMinBB,cloudMaxBB);
		mesh->getBoundingBox(meshMinBB,meshMaxBB);

		//max bounding-box (non-cubical)
		for (unsigned char k=0; k<3; ++k)
		{
			minBB.u[k] = std::min(meshMinBB.u[k],cloudMinBB.u[k]);
			maxBB.u[k] = std::max(meshMaxBB.u[k],cloudMaxBB.u[k]);
		}

		//max bounding-box (cubical)
		minCubifiedBB = minBB;
		maxCubifiedBB = maxBB;
		CCMiscTools::MakeMinAndMaxCubical(minCubifiedBB,maxCubifiedBB);
	}

	//compute the octree if necessary
	DgmOctree tempOctree(pointCloud);
	DgmOctree* octree = cloudOctree;
	bool rebuildTheOctree = false;
	if (!octree)
	{
		octree = &tempOctree;
		rebuildTheOctree = true;
	}
	else
	{
		//check the input octree dimensions
		const CCVector3& theOctreeMins = octree->getOctreeMins();
		const CCVector3& theOctreeMaxs = octree->getOctreeMaxs();
		for (unsigned char k=0; k<3; ++k)
		{
			if (	theOctreeMins.u[k] != minCubifiedBB.u[k]
				||	theOctreeMaxs.u[k] != maxCubifiedBB.u[k] )
			{
				rebuildTheOctree = true;
				break;
			}
		}
	}

	if (rebuildTheOctree)
	{
		//build the octree
		if (octree->build(minCubifiedBB,maxCubifiedBB,&cloudMinBB,&cloudMaxBB,progressCb) <= 0)
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_BUILD_OCTREE_FAILURE;
		}
	}

	OctreeAndMeshIntersection intersection;
	intersection.octree = octree;
	intersection.mesh = mesh;

	//we deduce the grid cell size very simply (as the bbox has been "cubified")
	PointCoordinateType cellSize = (maxCubifiedBB.x - minCubifiedBB.x) / (1 << params.octreeLevel);
	//we compute grid occupancy ... and we deduce the grid dimensions
	Tuple3ui gridSize;
	{
		for (unsigned char k=0; k<3; ++k)
		{
			intersection.minFillIndexes.u[k] = static_cast<int>(floor((minBB.u[k]-minCubifiedBB.u[k])/cellSize));
			intersection.maxFillIndexes.u[k] = static_cast<int>(floor((maxBB.u[k]-minCubifiedBB.u[k])/cellSize));
			gridSize.u[k] = static_cast<unsigned>(intersection.maxFillIndexes.u[k]-intersection.minFillIndexes.u[k]+1);
		}
	}

	//if the user (or the current cloud/mesh configuration) requires that we use a Distance Transform
	if (params.useDistanceMap)
	{
		intersection.distanceTransform = new SaitoSquaredDistanceTransform;
		if ( !intersection.distanceTransform || !intersection.distanceTransform->initGrid(gridSize))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_INIT_DISTANCE_TRANSFORM_GRID_FAILURE;
		}
		params.multiThread = false; //not necessary/supported
	}
	else
	{
		//we need to build the list of triangles intersecting each cell of the 3D grid
		if (!intersection.perCellTriangleList.init(gridSize.x, gridSize.y, gridSize.z, 0, nullptr))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_INIT_PER_CELL_TRIANGLE_LIST_FAILURE;
		}
	}

	//INTERSECT THE OCTREE WITH THE MESH
	int result = intersectMeshWithOctree(&intersection,params.octreeLevel,progressCb);
	if (result < DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		if (!(result == DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY ||
			result == DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER))
		{
			return result;
		}
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INTERSECT_MESH_WITH_OCTREE_FAILURE;
	}

	//reset the output distances
	pointCloud->enableScalarField();
	pointCloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

	//acceleration by approximating the distance
	if (params.useDistanceMap && intersection.distanceTransform)
	{
		if (!intersection.distanceTransform->propagateDistance(progressCb))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_PROPAGATE_DISTANCE_FAILURE;
		}
	}

	//WE CAN EVENTUALLY COMPUTE THE DISTANCES!
	result = computeCloud2MeshDistanceWithOctree(&intersection, params, progressCb);

	//don't forget to compute the square root of the (squared) unsigned distances
	if (result == DISTANCE_COMPUTATION_RESULTS::SUCCESS && 
		!params.signedDistances && 
		!params.useDistanceMap)
	{
		pointCloud->forEach(applySqrtToPointDist);
	}

	if (result < DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		if (!(result == DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY ||
			result == DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER))
		{
			return result;
		}
		return DISTANCE_COMPUTATION_RESULTS::ERROR_COMPUTE_CLOUD2_MESH_DISTANCE_WITH_OCTREE_FAILURE;
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

// Inspired from documents and code by:
// David Eberly
// Geometric Tools, LLC
// http://www.geometrictools.com/
ScalarType DistanceComputationTools::computePoint2TriangleDistance(const CCVector3* P, const GenericTriangle* theTriangle, bool signedDist, CCVector3* nearestP/*=0*/)
{
	assert(P && theTriangle);

	const CCVector3* A = theTriangle->_getA();
	const CCVector3* B = theTriangle->_getB();
	const CCVector3* C = theTriangle->_getC();

	//we do all computations with double precision, otherwise
	//some triangles with sharp angles will give very poor results.
	//slight precision improvement by casting to double prior to subtraction
	CCVector3d AP(static_cast<double>(P->x) - A->x, static_cast<double>(P->y) - A->y, static_cast<double>(P->z) - A->z);
	CCVector3d AB(static_cast<double>(B->x) - A->x, static_cast<double>(B->y) - A->y, static_cast<double>(B->z) - A->z);
	CCVector3d AC(static_cast<double>(C->x) - A->x, static_cast<double>(C->y) - A->y, static_cast<double>(C->z) - A->z);

	double a00 =  AB.dot(AB);
	double a01 =  AB.dot(AC);
	double a11 =  AC.dot(AC);
	double b0  = -AP.dot(AB);
	double b1  = -AP.dot(AC);
	double det = a00 * a11 - a01 * a01;
	double t0  = a01 * b1 - a11 * b0;
	double t1  = a01 * b0 - a00 * b1;

	if (t0 + t1 <= det)
	{
		if (t0 < 0)
		{
			if (t1 < 0)	 // region 4
			{
				if (b0 < 0)
				{
					t1 = 0;
					if (-b0 >= a00)	 // V0
					{
						t0 = 1.0;
					}
					else  // E01
					{
						t0 = -b0 / a00;
					}
				}
				else
				{
					t0 = 0;
					if (b1 >= 0)  // V0
					{
						t1 = 0;
					}
					else if (-b1 >= a11)  // V2
					{
						t1 = 1.0;
					}
					else  // E20
					{
						t1 = -b1 / a11;
					}
				}
			}
			else  // region 3
			{
				t0 = 0;
				if (b1 >= 0)  // V0
				{
					t1 = 0;
				}
				else if (-b1 >= a11)  // V2
				{
					t1 = 1.0;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < 0)  // region 5
		{
			t1 = 0;
			if (b0 >= 0)  // V0
			{
				t0 = 0;
			}
			else if (-b0 >= a00)  // V1
			{
				t0 = 1.0;
			}
			else  // E01
			{
				t0 = -b0 / a00;
			}
		}
		else  // region 0, interior
		{
			t0 /= det;
			t1 /= det;
		}
	}
	else
	{
		double tmp0;
		double tmp1;
		double numer;
		double denom;

		if (t0 < 0)	 // region 2
		{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V1
				{
					t0 = 1.0;
					t1 = 0;
				}
				else  // E12
				{
					t0 = numer / denom;
					t1 = 1.0 - t0;
				}
			}
			else
			{
				t0 = 0;
				if (tmp1 <= 0)	// V2
				{
					t1 = 1.0;
				}
				else if (b1 >= 0)  // V0
				{
					t1 = 0;
				}
				else  // E20
				{
					t1 = -b1 / a11;
				}
			}
		}
		else if (t1 < 0)  // region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V2
				{
					t1 = 1.0;
					t0 = 0;
				}
				else  // E12
				{
					t1 = numer / denom;
					t0 = 1.0 - t1;
				}
			}
			else
			{
				t1 = 0;
				if (tmp1 <= 0)	// V1
				{
					t0 = 1.0;
				}
				else if (b0 >= 0)  // V0
				{
					t0 = 0;
				}
				else  // E01
				{
					t0 = -b0 / a00;
				}
			}
		}
		else  // region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= 0)	 // V2
			{
				t0 = 0;
				t1 = 1.0;
			}
			else
			{
				denom = a00 - 2*a01 + a11;
				if (numer >= denom)	 // V1
				{
					t0 = 1.0;
					t1 = 0;
				}
				else  // 12
				{
					t0 = numer / denom;
					t1 = 1.0 - t0;
				}
			}
		}
	}

	//point on the plane (relative to A)
	CCVector3d Q = t0 * AB + t1 * AC;

	if (nearestP)
	{
		//if requested we also output the nearest point
		*nearestP = *A + CCVector3::fromArray(Q.u);
	}

	double squareDist = (Q - AP).norm2();
	if (signedDist)
	{
		ScalarType d = static_cast<ScalarType>(sqrt(squareDist));

		//triangle normal
		CCVector3d N = AB.cross(AC);

		//we test the sign of the cross product of the triangle normal and the vector AP
		return (AP.dot(N) < 0 ? -d : d);
	}
	else
	{
		return static_cast<ScalarType>(squareDist);
	}
}

ScalarType DistanceComputationTools::computePoint2PlaneDistance(const CCVector3* P,
																const PointCoordinateType* planeEquation)
{
	//point to plane distance: d = (a0*x+a1*y+a2*z - a3) / sqrt(a0^2+a1^2+a2^2)
	assert(std::abs(CCVector3::vnorm(planeEquation) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	return static_cast<ScalarType>((CCVector3::vdot(P->u, planeEquation) - planeEquation[3])/*/CCVector3::vnorm(planeEquation)*/); //norm == 1.0!
}

ScalarType DistanceComputationTools::computePoint2LineSegmentDistSquared(const CCVector3* p, const CCVector3* start, const CCVector3* end)
{
	assert(p && start && end);
	CCVector3 line = *end - *start;
	CCVector3 vec;
	ScalarType distSq;
	PointCoordinateType t = line.dot(*p - *start);
	PointCoordinateType normSq = line.norm2();
	if (normSq != 0) 
	{
		t /= normSq;
	}
	if (t < 0)
	{
		vec = *p - *start;
	}
	else if (t > 1)
	{
		vec = *p - *end;
	}
	else
	{
		CCVector3 pProjectedOnLine = *start + t * line;
		vec = *p - pProjectedOnLine;
	}

	distSq = vec.norm2();
	return distSq;
}

// This algorithm is a modification of the distance computation between a point and a cone from 
// Barbier & Galin's Fast Distance Computation Between a Point and Cylinders, Cones, Line Swept Spheres and Cone-Spheres.
// The modifications from the paper are to compute the closest distance when the point is interior to the cone.
// http://liris.cnrs.fr/Documents/Liris-1297.pdf
int DistanceComputationTools::computeCloud2ConeEquation(GenericIndexedCloudPersist* cloud, const CCVector3& coneP1, const CCVector3& coneP2, const PointCoordinateType coneR1, const PointCoordinateType coneR2, bool signedDistances/*=true*/, bool solutionType/*=false*/, double* rms/*=nullptr*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (coneR1 < coneR2)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_CONE_R1_LT_CONE_R2;
	}
	double dSumSq = 0.0;

	CCVector3 coneAxis = coneP2 - coneP1;
	double axisLength = coneAxis.normd();
	coneAxis.normalize();
	double delta = static_cast<double>(coneR2) - coneR1;
	double rr1 = static_cast<double>(coneR1) * coneR1;
	double rr2 = static_cast<double>(coneR2) * coneR2;
	double coneLength = sqrt((axisLength * axisLength) + (delta * delta));
	CCVector3d side{ axisLength, delta, 0.0};
	side /= coneLength;
	
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3 n = *P - coneP1;
		double x = n.dot(coneAxis);
		double xx = x * x;
		double yy = (n.norm2d()) - xx;
		double y = 0;
		double d = 0;
		double rx = 0; 
		double ry = 0;
		if (yy < 0)
		{
			yy = 0;
		}
		if(x <= 0) //Below the bottom point
		{
			if (yy < rr1)
			{
				if (!solutionType)
				{
					d = -x; //Below the bottom point within larger disk radius
				}
				else
				{
					d = 1; //Works
				}
			}
			else 
			{
				if (!solutionType)
				{
					y = sqrt(yy) - coneR1;
					d = sqrt((y * y) + xx); //Below the bottom point not within larger disk radius
				}
				else
				{
					d = 2; //Works
				}
			}

		}
		else //Above the bottom point
		{
			if (yy < rr2) // within smaller disk radius
			{
				if (x > axisLength) //outside cone within smaller disk
				{
					if (!solutionType)
					{
						d = x - axisLength; // Above the top point within top disk radius
					}
					else
					{
						d = 3;
					}
				}
				else //inside cone within smaller disk
				{
					if (!solutionType) 
					{
						y = sqrt(yy) - coneR1;
						ry = y * side[0] - x * side[1]; //rotated y value (distance from the coneside axis)
						d = std::min(axisLength - x, x); //determine whether closer to either radii 
						d = std::min(d, static_cast<double>(std::abs(ry))); //or to side
						d = -d; //negative inside
					}
					else
					{
						d = 4;
					}
				}
			}
			else // Outside smaller disk radius
			{
				y = sqrt(yy) - coneR1;
				{
					rx = y * side[1] + x * side[0]; //rotated x value (distance along the coneside axis)
					if (rx < 0)
					{
						if (!solutionType)
						{
							d = sqrt(y * y + xx);
						}
						else
						{
							d = 7; // point projects onto the large cap
						}
					}
					else
					{
						ry = y * side[0] - x * side[1]; //rotated y value (distance from the coneside axis)
						if (rx > coneLength)
						{
							if (!solutionType)
							{
								rx -= coneLength;
								d = sqrt(ry * ry + rx * rx);
							}
							else
							{
								d = 8; // point projects onto the small cap
							}
						}
						else
						{
							if (!solutionType)
							{
								d = ry; // point projects onto the side of cone
								if (ry < 0) 
								{//point is interior to the cone
									d = std::min(axisLength - x, x); //determine whether closer to either radii 
									d = std::min(d, static_cast<double>(std::abs(ry))); //or to side
									d = -d; //negative inside
								}
							}
							else
							{
								d = 9;
							}
						}
					}

				}
			}
		}
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

// This algorithm is a modification of the distance computation between a point and a cylinder from 
// Barbier & Galin's Fast Distance Computation Between a Point and Cylinders, Cones, Line Swept Spheres and Cone-Spheres.
// The modifications from the paper are to compute the closest distance when the point is interior to the capped cylinder.
// http://liris.cnrs.fr/Documents/Liris-1297.pdf
// solutionType 1 = exterior to the cylinder and within the bounds of the axis
// solutionType 2 = interior to the cylinder and either closer to an end-cap or the cylinder wall
// solutionType 3 = beyond the bounds of the cylinder's axis and radius
// solutionType 4 = beyond the bounds of the cylinder's axis but within the bounds of it's radius
int DistanceComputationTools::computeCloud2CylinderEquation(GenericIndexedCloudPersist* cloud, const CCVector3& cylinderP1, const CCVector3& cylinderP2, const PointCoordinateType cylinderRadius, bool signedDistances/*=true*/, bool solutionType/*=false*/, double* rms/*=0*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	double dSumSq = 0.0;

	CCVector3 cylinderCenter = (cylinderP1 + cylinderP2) / 2.;

	CCVector3 cylinderAxis = cylinderP2 - cylinderP1;
	double h = cylinderAxis.normd() / 2.;
	cylinderAxis.normalize();
	double cylinderRadius2 = static_cast<double>(cylinderRadius)* cylinderRadius;

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		CCVector3 n = *P - cylinderCenter;

		double x = std::abs(n.dot(cylinderAxis));
		double xx = x * x;
		double yy = (n.norm2d()) - xx;
		double y = 0;
		double d = 0;
		if (x <= h)
		{
			if (yy >= cylinderRadius2)
			{
				if (!solutionType)
				{
					d = sqrt(yy) - cylinderRadius; //exterior to the cylinder and within the bounds of the axis
				}
				else
				{
					d = 1;
				}
			}
			else
			{
				if (!solutionType)
				{
					d = -std::min(std::abs(sqrt(yy) - cylinderRadius), std::abs(h - x)); //interior to the cylinder and either closer to an end-cap or the cylinder wall
				}
				else
				{
					d = 2;
				}
			}
		}
		else
		{
			if (yy >= cylinderRadius2)
			{
				if (!solutionType) 
				{
					y = sqrt(yy);
					d = sqrt(((y - cylinderRadius) * (y - cylinderRadius)) + ((x - h) * (x - h))); //beyond the bounds of the cylinder's axis and radius
				}
				else
				{
					d = 3;
				}
			}
			else
			{
				if (!solutionType)
				{
					d = x - h; //beyond the bounds of the cylinder's axis but within the bounds of it's radius
				}
				else
				{
					d = 4;
				}
			}

		}

		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2SphereEquation(GenericIndexedCloudPersist *cloud, const CCVector3& sphereCenter, const PointCoordinateType sphereRadius, bool signedDistances/*=true*/, double* rms/*=0*/)
{
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	double dSumSq = 0.0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		double d = (*P - sphereCenter).normd() - sphereRadius;
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2PlaneEquation(GenericIndexedCloudPersist *cloud, const PointCoordinateType* planeEquation, bool signedDistances/*=true*/, double* rms/*=0*/)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	if (!planeEquation)
	{
		return DISTANCE_COMPUTATION_RESULTS::NULL_PLANE_EQUATION;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());
	if (norm2 < ZERO_TOLERANCE)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_PLANE_NORMAL_LT_ZERO;
	}
	double dSumSq = 0.0;
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* P = cloud->getPoint(i);
		double d = static_cast<double>(CCVector3::vdotd(P->u, planeEquation) - planeEquation[3]);
		if (signedDistances)
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
		}
		else
		{
			cloud->setPointScalarValue(i, static_cast<ScalarType>(std::abs(d)));
		}
		dSumSq += d * d;
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2RectangleEquation(GenericIndexedCloudPersist *cloud, PointCoordinateType widthX, PointCoordinateType widthY, const SquareMatrix& rotationTransform, const CCVector3& center, bool signedDist/*=true*/, double* rms/*= nullptr*/) 
{
	// p3---------------------p2
	// ^					  |
	// |e1					  |
	// |					  |
	// |		  e0		  |
	// p0-------------------->p1
	//Rect(s,t)=p0 + s*0e + t*e1 
	//for s,t in {0,1}
	assert(cloud);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0) 
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (widthX <= 0 || widthY <= 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_PRIMITIVE_DIMENSIONS;
	}
	
	CCVector3 widthXVec(widthX, 0, 0);
	CCVector3 widthYVec(0, widthY, 0);
	CCVector3 normalVector(0, 0, 1);

	widthXVec = rotationTransform * widthXVec;
	widthYVec = rotationTransform * widthYVec;
	normalVector = rotationTransform * normalVector;
	PointCoordinateType planeDistance = center.dot(normalVector);
	PointCoordinateType dSumSq = 0;
	CCVector3 rectangleP0 = center - (widthXVec / 2) - (widthYVec / 2);
	CCVector3 rectangleP1 = center + (widthXVec / 2) - (widthYVec / 2);
	CCVector3 rectangleP3 = center - (widthXVec / 2) + (widthYVec / 2);
	CCVector3 e0 = rectangleP1 - rectangleP0;
	CCVector3 e1 = rectangleP3 - rectangleP0;

	for (unsigned i = 0; i < count; ++i) 
	{
		const CCVector3* pe = cloud->getPoint(i);
		CCVector3 dist = (*pe - rectangleP0);
		PointCoordinateType s = e0.dot(dist);
		if (s > 0) 
		{
			PointCoordinateType dot0 = e0.norm2();
			if (s < dot0)
			{
				dist -= (s / dot0)*e0;
			}
			else
			{
				dist -= e0;
			}
		}

		PointCoordinateType t = e1.dot(dist);
		if (t > 0) 
		{
			PointCoordinateType dot1 = e1.norm2();
			if (t < dot1)
			{
				dist -= (t / dot1)*e1;
			}
			else
			{
				dist -= e1;
			}
		}
		PointCoordinateType d = static_cast<PointCoordinateType>(dist.normd());
		dSumSq += d * d;
		if (signedDist && pe->dot(normalVector) - planeDistance < 0)
		{			
			d = -d;
		}
		cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2BoxEquation(GenericIndexedCloudPersist* cloud, const CCVector3& boxDimensions, const SquareMatrix& rotationTransform, const CCVector3& boxCenter, bool signedDist/*=true*/, double* rms/*= nullptr*/) 
{
	assert(cloud);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (boxDimensions.x <= 0 || boxDimensions.y <= 0 || boxDimensions.z <= 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_INVALID_PRIMITIVE_DIMENSIONS;
	}
	// box half lengths hu hv and hw
	const PointCoordinateType hu = boxDimensions.x / 2;
	const PointCoordinateType hv = boxDimensions.y / 2;
	const PointCoordinateType hw = boxDimensions.z / 2;
	// box coordinates unit vectors u,v, and w
	CCVector3 u(1, 0, 0);
	CCVector3 v(0, 1, 0);
	CCVector3 w(0, 0, 1);
	u = rotationTransform * u;
	v = rotationTransform * v;
	w = rotationTransform * w;
	PointCoordinateType dSumSq = 0;
	for (unsigned i = 0; i < count; ++i) 
	{
		const CCVector3* p = cloud->getPoint(i);
		CCVector3 pointCenterDifference = (*p - boxCenter);
		CCVector3 p_inBoxCoords(pointCenterDifference.dot(u), pointCenterDifference.dot(v), pointCenterDifference.dot(w));
		bool insideBox = false;
		if (p_inBoxCoords.x > -hu && p_inBoxCoords.x < hu && p_inBoxCoords.y > -hv && p_inBoxCoords.y < hv && p_inBoxCoords.z > -hw && p_inBoxCoords.z < hw)
		{
			insideBox = true;
		}

		CCVector3 dist(0, 0, 0);
		if (p_inBoxCoords.x < -hu)
		{
			dist.x = -(p_inBoxCoords.x + hu);
		}
		else if (p_inBoxCoords.x > hu)
		{
			dist.x = p_inBoxCoords.x - hu;
		}
		else if (insideBox)
		{
			dist.x = std::abs(p_inBoxCoords.x) - hu;
		}

		if (p_inBoxCoords.y < -hv)
		{
			dist.y = -(p_inBoxCoords.y + hv);
		}
		else if (p_inBoxCoords.y > hv)
		{
			dist.y = p_inBoxCoords.y - hv;
		}
		else if (insideBox)
		{
			dist.y = std::abs(p_inBoxCoords.y) - hv;
		}

		if (p_inBoxCoords.z < -hw)
		{
			dist.z = -(p_inBoxCoords.z + hw);
		}
		else if (p_inBoxCoords.z > hw)
		{
			dist.z = p_inBoxCoords.z - hw;
		}
		else if (insideBox)
		{
			dist.z = std::abs(p_inBoxCoords.z) - hw;
		}

		if (insideBox)//take min distance inside box
		{ 
			if (dist.x >= dist.y && dist.x >= dist.z) 
			{
				dist.y = 0;
				dist.z = 0;
			}
			else if (dist.y >= dist.x && dist.y >= dist.z) 
			{
				dist.x = 0;
				dist.z = 0;
			}
			else if (dist.z >= dist.x && dist.z >= dist.y) 
			{
				dist.x = 0;
				dist.y = 0;
			}
		}
		PointCoordinateType d = static_cast<PointCoordinateType>(dist.normd());
		dSumSq += d * d;
		if (signedDist && insideBox)
		{
			d = -d;
		}
		cloud->setPointScalarValue(i, static_cast<ScalarType>(d));
	}
	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeCloud2PolylineEquation(GenericIndexedCloudPersist* cloud, const Polyline* polyline, double* rms /*= nullptr*/)
{
	assert(cloud);
	if (!cloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	unsigned count = cloud->size();
	if (count == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}
	if (!cloud->enableScalarField())
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	ScalarType d = 0;
	ScalarType dSumSq = 0;
	for (unsigned i = 0; i < count; ++i)
	{
		ScalarType distSq = NAN_VALUE;
		const CCVector3* p = cloud->getPoint(i);
		for (unsigned j = 0; j < polyline->size() - 1; j++)
		{
			const CCVector3* start = polyline->getPoint(j);
			const CCVector3* end = polyline->getPoint(j + 1);
			PointCoordinateType startXMinusA = start->x - p->x;
			PointCoordinateType startYMinusB = start->y - p->y;
			PointCoordinateType startZMinusC = start->z - p->z;
			PointCoordinateType endXMinusA = end->x - p->x;
			PointCoordinateType endYMinusB = end->y - p->y;
			PointCoordinateType endZMinusC = end->z - p->z;
			PointCoordinateType startXMinusASq = startXMinusA * startXMinusA;
			PointCoordinateType startYMinusBSq = startYMinusB * startYMinusB;
			PointCoordinateType startZMinusCSq = startZMinusC * startZMinusC;
			PointCoordinateType endXMinusASq = endXMinusA * endXMinusA;
			PointCoordinateType endYMinusBSq = endYMinusB * endYMinusB;
			PointCoordinateType endZMinusCSq = endZMinusC * endZMinusC;

			//Rejection test
			if (((startXMinusASq >= distSq) && (endXMinusASq >= distSq) && (startXMinusA * endXMinusA > ZERO_TOLERANCE)) ||
				((startYMinusBSq >= distSq) && (endYMinusBSq >= distSq) && (startYMinusB * endYMinusB > ZERO_TOLERANCE)) ||
				((startZMinusCSq >= distSq) && (endZMinusCSq >= distSq) && (startZMinusC * endZMinusC > ZERO_TOLERANCE)))
			{
				continue;
			}
			if (std::isnan(distSq)) 
			{
				distSq = computePoint2LineSegmentDistSquared(p, start, end);
			}
			else 
			{
				distSq = std::min(distSq, computePoint2LineSegmentDistSquared(p, start, end));
			}
		}
		d = sqrt(distSq);
		dSumSq += distSq;
		cloud->setPointScalarValue(i, d);
	}

	if (rms)
	{
		*rms = sqrt(dSumSq / count);
	}
	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

ScalarType DistanceComputationTools::computeCloud2PlaneDistanceRMS( GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
		return NAN_VALUE;
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	double dSumSq = 0.0;

	//compute deviations
	cloud->placeIteratorAtBeginning();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		double d = static_cast<double>(CCVector3::vdotd(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0
		
		dSumSq += d*d;
	}

	return static_cast<ScalarType>( sqrt(dSumSq/count) );
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneRobustMax(	GenericCloud* cloud,
																	const PointCoordinateType* planeEquation,
																	float percent)
{
	assert(cloud && planeEquation);
	assert(percent < 1.0f);

	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
	{
		return NAN_VALUE;
	}
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max @ 'percent'% (to avoid outliers)
	std::vector<PointCoordinateType> tail;
	std::size_t tailSize = static_cast<std::size_t>(ceil(static_cast<float>(count) * percent));
	tail.resize(tailSize);

	//compute deviations
	cloud->placeIteratorAtBeginning();
	std::size_t pos = 0;
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0

		if (pos < tailSize)
		{
			tail[pos++] = d;
		}
		else if (tail.back() < d)
		{
			tail.back() = d;
		}

		//search the max element of the tail
		std::size_t maxPos = pos-1;
		if (maxPos != 0)
		{
			std::size_t maxIndex = maxPos;
			for (std::size_t j=0; j<maxPos; ++j)
				if (tail[j] < tail[maxIndex])
					maxIndex = j;
			//and put it to the back!
			if (maxPos != maxIndex)
				std::swap(tail[maxIndex],tail[maxPos]);
		}
	}

	return static_cast<ScalarType>(tail.back());
}

ScalarType DistanceComputationTools::ComputeCloud2PlaneMaxDistance( GenericCloud* cloud,
																	const PointCoordinateType* planeEquation)
{
	assert(cloud && planeEquation);
	if (!cloud)
	{
		return 0;
	}
	//point count
	unsigned count = cloud->size();
	if (count == 0)
	{
		return 0;
	}

	//point to plane distance: d = std::abs(a0*x+a1*y+a2*z-a3) / sqrt(a0^2+a1^2+a2^2) <-- "norm"
	//but the norm should always be equal to 1.0!
	PointCoordinateType norm2 = CCVector3::vnorm2(planeEquation);
	if (norm2 < ZERO_TOLERANCE)
		return NAN_VALUE;
	assert(std::abs(sqrt(norm2) - PC_ONE) <= std::numeric_limits<PointCoordinateType>::epsilon());

	//we search the max distance
	PointCoordinateType maxDist = 0;
	
	cloud->placeIteratorAtBeginning();
	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = cloud->getNextPoint();
		PointCoordinateType d = std::abs(CCVector3::vdot(P->u,planeEquation) - planeEquation[3])/*/norm*/; //norm == 1.0
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

bool DistanceComputationTools::computeGeodesicDistances(GenericIndexedCloudPersist* cloud, unsigned seedPointIndex, unsigned char octreeLevel, GenericProgressCallback* progressCb)
{
	assert(cloud);
	if (!cloud)
	{
		return false;
	}
	unsigned n = cloud->size();
	if (n == 0 || seedPointIndex >= n)
		return false;

	cloud->enableScalarField();
	cloud->forEach(ScalarFieldTools::SetScalarValueToNaN);

	DgmOctree* octree = new DgmOctree(cloud);
	if (octree->build(progressCb) < 1)
	{
		delete octree;
		return false;
	}

	FastMarchingForPropagation fm;
	if (fm.init(cloud,octree,octreeLevel,true) < 0)
	{
		delete octree;
		return false;
	}

	//on cherche la cellule de l'octree qui englobe le "seedPoint"
	Tuple3i cellPos;
	octree->getTheCellPosWhichIncludesThePoint(cloud->getPoint(seedPointIndex), cellPos, octreeLevel);
	fm.setSeedCell(cellPos);

	bool result = false;
	if (fm.propagate() >= 0)
		result = fm.setPropagationTimingsAsDistances();

	delete octree;
	octree = nullptr;

	return result;
}

int DistanceComputationTools::diff( GenericIndexedCloudPersist* comparedCloud,
									GenericIndexedCloudPersist* referenceCloud,
									GenericProgressCallback* progressCb)
{
	if (!comparedCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}

	if (!referenceCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}

	unsigned nA = comparedCloud->size();
	if (nA == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_COMPAREDCLOUD;
	}

	if (referenceCloud->size() == 0)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_EMPTY_REFERENCECLOUD;
	}
	//Reference cloud to store closest point set
	ReferenceCloud A_in_B(referenceCloud);

	Cloud2CloudDistanceComputationParams params;
	params.octreeLevel = DgmOctree::MAX_OCTREE_LEVEL-1;
	params.CPSet = &A_in_B;

	int result = computeCloud2CloudDistance(comparedCloud,referenceCloud,params,progressCb);
	if (result < DISTANCE_COMPUTATION_RESULTS::SUCCESS)
	{
		if (!(result == DISTANCE_COMPUTATION_RESULTS::ERROR_OUT_OF_MEMORY ||
			result == DISTANCE_COMPUTATION_RESULTS::CANCELED_BY_USER))
		{
			return result;
		}
		return DISTANCE_COMPUTATION_RESULTS::ERROR_COMPUTE_CLOUD2_CLOUD_DISTANCE_FAILURE;
	}

	for (unsigned i=0; i<nA; ++i)
	{
		ScalarType dA = comparedCloud->getPointScalarValue(i);
		ScalarType dB = A_in_B.getPointScalarValue(i);

		//handle invalid values
		comparedCloud->setPointScalarValue(i,ScalarField::ValidValue(dA) && ScalarField::ValidValue(dB) ? dA-dB : NAN_VALUE);
	}

	return DISTANCE_COMPUTATION_RESULTS::SUCCESS;
}

int DistanceComputationTools::computeApproxCloud2CloudDistance( GenericIndexedCloudPersist* comparedCloud,
																GenericIndexedCloudPersist* referenceCloud,
																unsigned char octreeLevel,
																PointCoordinateType maxSearchDist/*=-PC_ONE*/,
																GenericProgressCallback* progressCb/*=0*/,
																DgmOctree* compOctree/*=0*/,
																DgmOctree* refOctree/*=0*/)
{
	if (!comparedCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_COMPAREDCLOUD;
	}
	if (!referenceCloud)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_NULL_REFERENCECLOUD;
	}
	if (octreeLevel < 1)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_LEVEL_LT_ONE;
	}
	if (octreeLevel > DgmOctree::MAX_OCTREE_LEVEL)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_OCTREE_LEVEL_GT_MAX_OCTREE_LEVEL;
	}
	//compute octrees with the same bounding-box
	DgmOctree *octreeA = compOctree;
	DgmOctree *octreeB = refOctree;
	if (synchronizeOctrees(comparedCloud, referenceCloud, octreeA, octreeB, maxSearchDist, progressCb) != SYNCHRONIZED)
	{
		return DISTANCE_COMPUTATION_RESULTS::ERROR_SYNCHRONIZE_OCTREES_FAILURE;
	}
	const int* minIndexesA = octreeA->getMinFillIndexes(octreeLevel);
	const int* maxIndexesA = octreeA->getMaxFillIndexes(octreeLevel);
	const int* minIndexesB = octreeB->getMinFillIndexes(octreeLevel);
	const int* maxIndexesB = octreeB->getMaxFillIndexes(octreeLevel);

	Tuple3i minIndexes( std::min(minIndexesA[0], minIndexesB[0]),
						std::min(minIndexesA[1], minIndexesB[1]),
						std::min(minIndexesA[2], minIndexesB[2]) );
	Tuple3i maxIndexes( std::max(maxIndexesA[0], maxIndexesB[0]),
						std::max(maxIndexesA[1], maxIndexesB[1]),
						std::max(maxIndexesA[2], maxIndexesB[2]) );
	
	Tuple3ui boxSize(	static_cast<unsigned>(maxIndexes.x - minIndexes.x + 1),
						static_cast<unsigned>(maxIndexes.y - minIndexes.y + 1),
						static_cast<unsigned>(maxIndexes.z - minIndexes.z + 1) );

	if (!comparedCloud->enableScalarField())
	{
		//not enough memory
		return DISTANCE_COMPUTATION_RESULTS::ERROR_ENABLE_SCALAR_FIELD_FAILURE;
	}
	if (maxSearchDist > 0)
	{
		//if maxSearchDist is defined, we might skip some points
		//so we set a default distance for all of them
		const ScalarType resetValue = static_cast<ScalarType>(maxSearchDist);
		for (unsigned i = 0; i < comparedCloud->size(); ++i)
		{
			comparedCloud->setPointScalarValue(i, resetValue);
		}
	}

	int result = DISTANCE_COMPUTATION_RESULTS::SUCCESS;

	//instantiate the Distance Transform grid
	SaitoSquaredDistanceTransform dtGrid;
	if (dtGrid.initGrid(boxSize))
	{
		//project the (filled) cells of octree B in the DT grid
		{
			DgmOctree::cellCodesContainer theCodes;
			if (!octreeB->getCellCodes(octreeLevel, theCodes, true))
			{
				return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_CODES_FAILURE;
			}

			while (!theCodes.empty())
			{
				DgmOctree::CellCode theCode = theCodes.back();
				theCodes.pop_back();
				Tuple3i cellPos;
				octreeB->getCellPos(theCode, octreeLevel, cellPos, true);
				cellPos -= minIndexes;
				dtGrid.setValue(cellPos, 1);
			}
		}

		//propagate the Distance Transform over the grid
		if (!dtGrid.propagateDistance(progressCb))
		{
			return DISTANCE_COMPUTATION_RESULTS::ERROR_PROPAGATE_DISTANCE_FAILURE;
		}

		//eventually get the approx. distance for each cell of octree A
		//and assign it to the points inside
		ScalarType cellSize = static_cast<ScalarType>(octreeA->getCellSize(octreeLevel));

		DgmOctree::cellIndexesContainer theIndexes;
		if (!octreeA->getCellIndexes(octreeLevel, theIndexes))
		{
			//not enough memory
			if (!compOctree)
				delete octreeA;
			if (!refOctree)
				delete octreeB;
			return DISTANCE_COMPUTATION_RESULTS::ERROR_GET_CELL_INDEXES_FAILURE;
		}

		ScalarType maxD = 0;
		ReferenceCloud Yk(octreeA->associatedCloud());

		while (!theIndexes.empty())
		{
			unsigned theIndex = theIndexes.back();
			theIndexes.pop_back();

			Tuple3i cellPos;
			octreeA->getCellPos(octreeA->getCellCode(theIndex),octreeLevel,cellPos,false);
			cellPos -= minIndexes;
			unsigned di = dtGrid.getValue(cellPos);
			ScalarType d = sqrt(static_cast<ScalarType>(di)) * cellSize;
			if (d > maxD)
			{
				maxD = d;
			}
			
			//the maximum distance is 'maxSearchDist' (if defined)
			if (maxSearchDist <= 0 || d < maxSearchDist)
			{
				if (!octreeA->getPointsInCellByCellIndex(&Yk, theIndex, octreeLevel))
				{
					return DISTANCE_COMPUTATION_RESULTS::ERROR_EXECUTE_GET_POINTS_IN_CELL_BY_INDEX_FAILURE;
				}
				for (unsigned j = 0; j < Yk.size(); ++j)
				{
					Yk.setPointScalarValue(j, d);
				}
			}
		}

		result = static_cast<int>(maxD);
	}
	else //DT grid init failed
	{
		result = DISTANCE_COMPUTATION_RESULTS::ERROR_INIT_DISTANCE_TRANSFORM_GRID_FAILURE;
	}

	if (!compOctree)
	{
		delete octreeA;
		octreeA = nullptr;
	}
	if (!refOctree)
	{
		delete octreeB;
		octreeB = nullptr;
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
