//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <CloudSamplingTools.h>

//local
#include <DgmOctreeReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <Neighbourhood.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <SimpleMesh.h>

//system
#include <algorithm>
#include <random>

using namespace CCLib;

GenericIndexedCloud* CloudSamplingTools::resampleCloudWithOctree(	GenericIndexedCloudPersist* inputCloud,
																	int newNumberOfPoints,
																	RESAMPLING_CELL_METHOD resamplingMethod,
																	GenericProgressCallback* progressCb,
																	DgmOctree* inputOctree)
{
	assert(inputCloud);

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
			return nullptr;
	}

	//look for the Octree level that gives the number of cells (= points) closest to the desired value
	unsigned char bestLevel = octree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	GenericIndexedCloud* sampledCloud = resampleCloudWithOctreeAtLevel(inputCloud, bestLevel, resamplingMethod, progressCb, octree);

	if (!inputOctree)
		delete octree;

	return sampledCloud;
}

PointCloud* CloudSamplingTools::resampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* inputCloud,
																unsigned char octreeLevel,
																RESAMPLING_CELL_METHOD resamplingMethod,
																GenericProgressCallback* progressCb/*=0*/,
																DgmOctree* inputOctree/*=0*/)
{
	assert(inputCloud);

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return nullptr;
		}
	}

	PointCloud* cloud = new PointCloud();

	unsigned nCells = octree->getCellNumber(octreeLevel);
	if (!cloud->reserve(nCells))
	{
		if (!inputOctree)
			delete octree;
		delete cloud;
		return nullptr;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2] = {	reinterpret_cast<void*>(cloud),
										reinterpret_cast<void*>(&resamplingMethod) };

	if (octree->executeFunctionForAllCellsAtLevel(	octreeLevel,
													&resampleCellAtLevel,
													additionalParameters,
													false, //the process is so simple that MT is slower!
													progressCb,
													"Cloud Resampling") == 0)
	{
		//something went wrong
		delete cloud;
		cloud = nullptr;

	}

	if (!inputOctree)
		delete octree;

	return cloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctree(	GenericIndexedCloudPersist* inputCloud,
																int newNumberOfPoints,
																SUBSAMPLING_CELL_METHOD subsamplingMethod,
																GenericProgressCallback* progressCb/*=0*/,
																DgmOctree* inputOctree/*=0*/)
{
	assert(inputCloud);

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
			return nullptr;
	}

	//on cherche le niveau qui donne le nombre de points le plus proche de la consigne
	unsigned char bestLevel = octree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	ReferenceCloud* subsampledCloud = subsampleCloudWithOctreeAtLevel(inputCloud,bestLevel,subsamplingMethod,progressCb,octree);

	if (!inputOctree)
		delete octree;

	return subsampledCloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* inputCloud,
																	unsigned char octreeLevel,
																	SUBSAMPLING_CELL_METHOD subsamplingMethod,
																	GenericProgressCallback* progressCb/*=0*/,
																	DgmOctree* inputOctree/*=0*/)
{
	assert(inputCloud);

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return nullptr;
		}
	}

	ReferenceCloud* cloud = new ReferenceCloud(inputCloud);

	unsigned nCells = octree->getCellNumber(octreeLevel);
	if (!cloud->reserve(nCells))
	{
		if (!inputOctree)
			delete octree;
		delete cloud;
		return nullptr;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2] = {	reinterpret_cast<void*>(cloud),
										reinterpret_cast<void*>(&subsamplingMethod) };

	if (octree->executeFunctionForAllCellsAtLevel(	octreeLevel,
													&subsampleCellAtLevel,
													additionalParameters,
													false, //the process is so simple that MT is slower!
													progressCb,
													"Cloud Subsampling") == 0)
	{
		//something went wrong
		delete cloud;
		cloud = nullptr;
	}

	if (!inputOctree)
		delete octree;

	return cloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudRandomly(GenericIndexedCloudPersist* inputCloud, unsigned newNumberOfPoints, GenericProgressCallback* progressCb/*=0*/)
{
	assert(inputCloud);

	unsigned theCloudSize = inputCloud->size();

	//we put all input points in a ReferenceCloud
	ReferenceCloud* newCloud = new ReferenceCloud(inputCloud);
	if (!newCloud->addPointIndex(0,theCloudSize))
	{
		delete newCloud;
		return nullptr;
	}

	//we have less points than requested?!
	if (theCloudSize <= newNumberOfPoints)
	{
		return newCloud;
	}

	unsigned pointsToRemove = theCloudSize - newNumberOfPoints;
	std::random_device rd;   // non-deterministic generator
	std::mt19937 gen(rd());  // to seed mersenne twister.

	NormalizedProgress normProgress(progressCb, pointsToRemove);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo("Random subsampling");
		}
		progressCb->update(0);
		progressCb->start();
	}

	//we randomly remove "inputCloud.size() - newNumberOfPoints" points (much simpler)
	unsigned lastPointIndex = theCloudSize-1;
	for (unsigned i=0; i<pointsToRemove; ++i)
	{
		std::uniform_int_distribution<unsigned> dist(0, lastPointIndex);
		unsigned index = dist(gen);
		newCloud->swap(index,lastPointIndex);
		--lastPointIndex;

		if (progressCb && !normProgress.oneStep())
		{
			//cancel process
			delete newCloud;
			return nullptr;
		}
	}

	newCloud->resize(newNumberOfPoints); //always smaller, so it should be ok!

	return newCloud;
}

ReferenceCloud* CloudSamplingTools::resampleCloudSpatially(GenericIndexedCloudPersist* inputCloud,
															PointCoordinateType minDistance,
															const SFModulationParams& modParams,
															DgmOctree* inputOctree/*=0*/,
															GenericProgressCallback* progressCb/*=0*/)
{
	assert(inputCloud);
	unsigned cloudSize = inputCloud->size();

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build() < static_cast<int>(cloudSize))
		{
			delete octree;
			return nullptr;
		}
	}
	assert(octree && octree->associatedCloud() == inputCloud);

	//output cloud
	ReferenceCloud* sampledCloud = new ReferenceCloud(inputCloud);
	const unsigned c_reserveStep = 65536;
	if (!sampledCloud->reserve(std::min(cloudSize, c_reserveStep)))
	{
		if (!inputOctree)
			delete octree;
		return nullptr;
	}

	std::vector<char> markers; //DGM: upgraded from vector, as this can be quite huge!
	try
	{
		markers.resize(cloudSize, 1); //true by default
	}
	catch (const std::bad_alloc&)
	{
		if (!inputOctree)
			delete octree;
		delete sampledCloud;
		return nullptr;
	}

	//best octree level (there may be several of them if we use parameter modulation)
	std::vector<unsigned char> bestOctreeLevel;
	bool modParamsEnabled = modParams.enabled;
	ScalarType sfMin = 0;
	ScalarType sfMax = 0;
	try
	{
		if (modParams.enabled)
		{
			//compute min and max sf values
			ScalarFieldTools::computeScalarFieldExtremas(inputCloud, sfMin, sfMax);

			if (!ScalarField::ValidValue(sfMin))
			{
				//all SF values are NAN?!
				modParamsEnabled = false;
			}
			else
			{
				//compute min and max 'best' levels
				PointCoordinateType dist0 = static_cast<PointCoordinateType>(sfMin * modParams.a + modParams.b);
				PointCoordinateType dist1 = static_cast<PointCoordinateType>(sfMax * modParams.a + modParams.b);
				unsigned char level0 = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(dist0);
				unsigned char level1 = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(dist1);

				bestOctreeLevel.push_back(level0);
				if (level1 != level0)
				{
					//add intermediate levels if necessary
					std::size_t levelCount = (level1 < level0 ? level0 - level1 : level1 - level0) + 1;
					assert(levelCount != 0);

					for (std::size_t i = 1; i < levelCount - 1; ++i) //we already know level0 and level1!
					{
						ScalarType sfVal = sfMin + i*((sfMax - sfMin) / levelCount);
						PointCoordinateType dist = static_cast<PointCoordinateType>(sfVal * modParams.a + modParams.b);
						unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(dist);
						bestOctreeLevel.push_back(level);
					}
				}
				bestOctreeLevel.push_back(level1);
			}
		}
		else
		{
			unsigned char defaultLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(minDistance);
			bestOctreeLevel.push_back(defaultLevel);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		if (!inputOctree)
		{
			delete octree;
		}
		delete sampledCloud;
		return nullptr;
	}

	//progress notification
	NormalizedProgress normProgress(progressCb, cloudSize);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Spatial resampling");
			char buffer[256];
			sprintf(buffer, "Points: %u\nMin dist.: %f", cloudSize, minDistance);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	//for each point in the cloud that is still 'marked', we look
	//for its neighbors and remove their own marks
	bool error = false;
	//default octree level
	assert(!bestOctreeLevel.empty());
	unsigned char octreeLevel = bestOctreeLevel.front();
	//default distance between points
	PointCoordinateType minDistBetweenPoints = minDistance;
	for (unsigned i = 0; i < cloudSize; i++)
	{
		//no mark? we skip this point
		if (markers[i] != 0)
		{
			//init neighbor search structure
			const CCVector3* P = inputCloud->getPoint(i);

			//parameters modulation
			if (modParamsEnabled)
			{
				ScalarType sfVal = inputCloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(sfVal))
				{
					//modulate minDistance
					minDistBetweenPoints = static_cast<PointCoordinateType>(sfVal * modParams.a + modParams.b);
					//get (approximate) best level
					std::size_t levelIndex = static_cast<std::size_t>(bestOctreeLevel.size() * ((sfVal - sfMin) / (sfMax - sfMin)));
					if (levelIndex == bestOctreeLevel.size())
						--levelIndex;
					octreeLevel = bestOctreeLevel[levelIndex];
				}
				else
				{
					minDistBetweenPoints = minDistance;
					octreeLevel = bestOctreeLevel.front();
				}
			}

			//look for neighbors and 'de-mark' them
			{
				DgmOctree::NeighboursSet neighbours;
				octree->getPointsInSphericalNeighbourhood(*P, minDistBetweenPoints, neighbours, octreeLevel);
				for (DgmOctree::NeighboursSet::iterator it = neighbours.begin(); it != neighbours.end(); ++it)
					if (it->pointIndex != i)
						markers[it->pointIndex] = 0;
			}

			//At this stage, the ith point is the only one marked in a radius of <minDistance>.
			//Therefore it will necessarily be in the final cloud!
			if (sampledCloud->size() == sampledCloud->capacity() && !sampledCloud->reserve(sampledCloud->capacity() + c_reserveStep))
			{
				//not enough memory
				error = true;
				break;
			}
			if (!sampledCloud->addPointIndex(i))
			{
				//not enough memory
				error = true;
				break;
			}
		}

		//progress indicator
		if (progressCb && !normProgress.oneStep())
		{
			//cancel process
			error = true;
			break;
		}
	}

	//remove unnecessarily allocated memory
	if (!error)
	{
		if (sampledCloud->capacity() > sampledCloud->size())
			sampledCloud->resize(sampledCloud->size());
	}
	else
	{
		delete sampledCloud;
		sampledCloud = nullptr;
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	if (!inputOctree)
	{
		//locally computed octree
		delete octree;
		octree = nullptr;
	}

	return sampledCloud;
}

ReferenceCloud* CloudSamplingTools::sorFilter(	GenericIndexedCloudPersist* inputCloud,
												int knn/*=6*/,
												double nSigma/*=1.0*/,
												DgmOctree* inputOctree/*=0*/,
												GenericProgressCallback* progressCb/*=0*/)
{
	if (!inputCloud || knn <= 0 || inputCloud->size() <= static_cast<unsigned>(knn))
	{
		//invalid input
		assert(false);
		return nullptr;
	}

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		//compute the octree if necessary
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return nullptr;
		}
	}

	//output
	ReferenceCloud* filteredCloud = nullptr;

	for (unsigned step = 0; step < 1; ++step) //fake loop for easy break
	{
		unsigned pointCount = inputCloud->size();

		std::vector<PointCoordinateType> meanDistances;
		try
		{
			meanDistances.resize(pointCount, 0);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			break;
		}
		
		double avgDist = 0;
		double stdDev = 0;

		//1st step: compute the average distance to the neighbors
		{
			//additional parameters
			void* additionalParameters[] = {reinterpret_cast<void*>(&knn),
											reinterpret_cast<void*>(&meanDistances)
			};

			unsigned char octreeLevel = octree->findBestLevelForAGivenPopulationPerCell(knn);

			if (octree->executeFunctionForAllCellsAtLevel(	octreeLevel,
															&applySORFilterAtLevel,
															additionalParameters,
															true,
															progressCb,
															"SOR filter") == 0)
			{
				//something went wrong
				break;
			}

			//deduce the average distance and std. dev.
			double sumDist = 0;
			double sumSquareDist = 0;
			for (unsigned i = 0; i < pointCount; ++i)
			{
				sumDist += meanDistances[i];
				sumSquareDist += meanDistances[i] * meanDistances[i];
			}
			avgDist = sumDist / pointCount;
			stdDev = sqrt(std::abs(sumSquareDist / pointCount - avgDist*avgDist));
		}

		//2nd step: remove the farthest points 
		{
			//deduce the max distance
			double maxDist = avgDist + nSigma * stdDev;

			filteredCloud = new ReferenceCloud(inputCloud);
			if (!filteredCloud->reserve(pointCount))
			{
				//not enough memory
				delete filteredCloud;
				filteredCloud = nullptr;
				break;
			}

			for (unsigned i = 0; i < pointCount; ++i)
			{
				if (meanDistances[i] <= maxDist)
				{
					filteredCloud->addPointIndex(i);
				}
			}

			filteredCloud->resize(filteredCloud->size());
		}
	}

	if (!inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	return filteredCloud;
}

ReferenceCloud* CloudSamplingTools::noiseFilter(GenericIndexedCloudPersist* inputCloud,
												PointCoordinateType kernelRadius,
												double nSigma,
												bool removeIsolatedPoints/*=false*/,
												bool useKnn/*=false*/,
												int knn/*=6*/,
												bool useAbsoluteError/*=true*/,
												double absoluteError/*=0.0*/,
												DgmOctree* inputOctree/*=0*/,
												GenericProgressCallback* progressCb/*=0*/)
{
	if (!inputCloud || inputCloud->size() < 2 || (useKnn && knn <= 0) || (!useKnn && kernelRadius <= 0))
	{
		//invalid input
		assert(false);
		return nullptr;
	}

	DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new DgmOctree(inputCloud);
		if (octree->build(progressCb) < 1)
		{
			delete octree;
			return nullptr;
		}
	}

	ReferenceCloud* filteredCloud = new ReferenceCloud(inputCloud);

	unsigned pointCount = inputCloud->size();
	if (!filteredCloud->reserve(pointCount))
	{
		//not enough memory
		if (!inputOctree)
			delete octree;
		delete filteredCloud;
		return nullptr;
	}

	//additional parameters
	void* additionalParameters[] = {reinterpret_cast<void*>(filteredCloud),
									reinterpret_cast<void*>(&kernelRadius),
									reinterpret_cast<void*>(&nSigma),
									reinterpret_cast<void*>(&removeIsolatedPoints),
									reinterpret_cast<void*>(&useKnn),
									reinterpret_cast<void*>(&knn),
									reinterpret_cast<void*>(&useAbsoluteError),
									reinterpret_cast<void*>(&absoluteError)
	};

	unsigned char octreeLevel = 0;
	if (useKnn)
		octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(kernelRadius);
	else
		octreeLevel = octree->findBestLevelForAGivenPopulationPerCell(knn);

	if (octree->executeFunctionForAllCellsAtLevel(	octreeLevel,
													&applyNoiseFilterAtLevel,
													additionalParameters,
													true,
													progressCb,
													"Noise filter" ) == 0)
	{
		//something went wrong
		delete filteredCloud;
		filteredCloud = nullptr;
	}

	if (!inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	if (filteredCloud)
	{
		filteredCloud->resize(filteredCloud->size());
	}

	return filteredCloud;
}

bool CloudSamplingTools::resampleCellAtLevel(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress/*=0*/)
{
	PointCloud* cloud						= static_cast<PointCloud*>(additionalParameters[0]);
	RESAMPLING_CELL_METHOD resamplingMethod	= *static_cast<RESAMPLING_CELL_METHOD*>(additionalParameters[1]);

	if (resamplingMethod == CELL_GRAVITY_CENTER)
	{
		const CCVector3* P = Neighbourhood(cell.points).getGravityCenter();
		if (!P)
			return false;
		cloud->addPoint(*P);
	}
	else //if (resamplingMethod == CELL_CENTER)
	{
		CCVector3 center;
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);
		cloud->addPoint(center);
	}

	if (nProgress && !nProgress->steps(cell.points->size()))
	{
		return false;
	}

	return true;
}

bool CloudSamplingTools::subsampleCellAtLevel(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress/*=0*/)
{
	ReferenceCloud* cloud						= static_cast<ReferenceCloud*>(additionalParameters[0]);
	SUBSAMPLING_CELL_METHOD subsamplingMethod	= *static_cast<SUBSAMPLING_CELL_METHOD*>(additionalParameters[1]);

	unsigned selectedPointIndex = 0;
	unsigned pointsCount = cell.points->size();

	if (subsamplingMethod == RANDOM_POINT)
	{
		selectedPointIndex = (static_cast<unsigned>(rand()) % pointsCount);

		if (nProgress && !nProgress->steps(pointsCount))
		{
			return false;
		}
	}
	else // if (subsamplingMethod == NEAREST_POINT_TO_CELL_CENTER)
	{
		CCVector3 center;
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);

		PointCoordinateType minSquareDist = (*cell.points->getPoint(0) - center).norm2();

		for (unsigned i=1; i<pointsCount; ++i)
		{
			PointCoordinateType squareDist = (*cell.points->getPoint(i) - center).norm2();
			if (squareDist < minSquareDist)
			{
				selectedPointIndex = i;
				minSquareDist = squareDist;
			}

			if (nProgress && !nProgress->oneStep())
			{
				return false;
			}
		}
	}

	return cloud->addPointIndex(cell.points->getPointGlobalIndex(selectedPointIndex));
}

bool CloudSamplingTools::applyNoiseFilterAtLevel(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress/*=0*/)
{
	ReferenceCloud* cloud				=  static_cast<ReferenceCloud*>(additionalParameters[0]);
	PointCoordinateType kernelRadius	= *static_cast<PointCoordinateType*>(additionalParameters[1]);
	double nSigma						= *static_cast<double*>(additionalParameters[2]);
	bool removeIsolatedPoints			= *static_cast<bool*>(additionalParameters[3]);
	bool useKnn							= *static_cast<bool*>(additionalParameters[4]);
	int knn								= *static_cast<int*>(additionalParameters[5]);
	bool useAbsoluteError				= *static_cast<bool*>(additionalParameters[6]);
	double absoluteError				= *static_cast<double*>(additionalParameters[7]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(kernelRadius, cell.parentOctree->getCellSize(nNSS.level));
	if (useKnn)
	{
		nNSS.minNumberOfNeighbors = knn;
	}
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);

		//look for neighbors (either inside a sphere or the k nearest ones)
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
		unsigned neighborCount = 0;

		if (useKnn)
			neighborCount = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS);
		else
			neighborCount = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, kernelRadius, false);

		if (neighborCount > 3) //we want 3 points or more (other than the point itself!)
		{
			//find the query point in the nearest neighbors set and place it at the end
			const unsigned globalIndex = cell.points->getPointGlobalIndex(i);
			unsigned localIndex = 0;
			while (localIndex < neighborCount && nNSS.pointsInNeighbourhood[localIndex].pointIndex != globalIndex)
				++localIndex;
			//the query point should be in the nearest neighbors set!
			assert(localIndex < neighborCount);
			if (localIndex + 1 < neighborCount) //no need to swap with another point if it's already at the end!
			{
				std::swap(nNSS.pointsInNeighbourhood[localIndex], nNSS.pointsInNeighbourhood[neighborCount - 1]);
			}

			unsigned realNeighborCount = neighborCount - 1;
			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood, realNeighborCount); //we don't take the query point into account!
			Neighbourhood Z(&neighboursCloud);

			const PointCoordinateType* lsPlane = Z.getLSPlane();
			if (lsPlane)
			{
				double maxD = absoluteError;
				if (!useAbsoluteError)
				{
					//compute the std. dev. to this plane
					double sum_d = 0;
					double sum_d2 = 0;
					for (unsigned j = 0; j < realNeighborCount; ++j)
					{
						const CCVector3* P = neighboursCloud.getPoint(j);
						double d = CCLib::DistanceComputationTools::computePoint2PlaneDistance(P, lsPlane);
						sum_d += d;
						sum_d2 += d*d;
					}

					double stddev = sqrt(std::abs(sum_d2*realNeighborCount - sum_d*sum_d)) / realNeighborCount;
					maxD = stddev * nSigma;
				}

				//distance from the query point to the plane
				double d = std::abs(CCLib::DistanceComputationTools::computePoint2PlaneDistance(&nNSS.queryPoint, lsPlane));

				if (d <= maxD)
				{
					cloud->addPointIndex(globalIndex);
				}
			}
			else
			{
				//TODO: ???
			}
		}
		else
		{
			//not enough points to fit a plane AND compute distances to it
			if (!removeIsolatedPoints)
			{
				//we keep the point
				unsigned globalIndex = cell.points->getPointGlobalIndex(i);
				cloud->addPointIndex(globalIndex);
			}
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

bool CloudSamplingTools::applySORFilterAtLevel(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress/*=0*/)
{
	int knn											= *static_cast<int*>(additionalParameters[0]);
	std::vector<PointCoordinateType>& meanDistances = *static_cast<std::vector<PointCoordinateType>*>(additionalParameters[1]);

	//structure for nearest neighbors search
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.minNumberOfNeighbors = knn; //DGM: I woud have put knn+1 (as the point itself will be ignored) but in this case we won't get the same result as PCL!
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		cell.points->getPoint(i, nNSS.queryPoint);
		const unsigned globalIndex = cell.points->getPointGlobalIndex(i);

		//look for the k nearest neighbors
		cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS);
		double sumDist = 0;
		unsigned count = 0;
		for (int j = 0; j < knn; ++j)
		{
			if (nNSS.pointsInNeighbourhood[j].pointIndex != globalIndex)
			{
				sumDist += sqrt(nNSS.pointsInNeighbourhood[j].squareDistd);
				++count;
			}
		}

		if (count)
		{
			meanDistances[globalIndex] = static_cast<PointCoordinateType>(sumDist / count);
		}
		else
		{
			//shouldn't happen
			assert(false);
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}
