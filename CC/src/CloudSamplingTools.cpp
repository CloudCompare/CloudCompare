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

#include "CloudSamplingTools.h"

//local
#include "GenericIndexedCloudPersist.h"
#include "GenericIndexedMesh.h"
#include "SimpleCloud.h"
#include "ReferenceCloud.h"
#include "Neighbourhood.h"
#include "SimpleMesh.h"
#include "GenericProgressCallback.h"

//system
#include <assert.h>

using namespace CCLib;

GenericIndexedCloud* CloudSamplingTools::resampleCloudWithOctree(GenericIndexedCloudPersist* theCloud, int newNumberOfPoints, RESAMPLING_CELL_METHOD resamplingMethod, GenericProgressCallback* progressCb, DgmOctree* _theOctree)
{
	assert(theCloud);

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
			return 0;
	}

	//on cherche le niveau qui donne le nombre de points le plus proche de la consigne
	uchar bestLevel=theOctree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	GenericIndexedCloud* sampledCloud = resampleCloudWithOctreeAtLevel(theCloud,bestLevel,resamplingMethod,progressCb,theOctree);

	if (!_theOctree)
		delete theOctree;

	return sampledCloud;
}

SimpleCloud* CloudSamplingTools::resampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud, uchar octreeLevel, RESAMPLING_CELL_METHOD resamplingMethod, GenericProgressCallback* progressCb, DgmOctree* _theOctree)
{
	assert(theCloud);

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return 0;
		}
	}

	SimpleCloud* cloud = new SimpleCloud();

	unsigned nCells = theOctree->getCellNumber(octreeLevel);
	if (!cloud->reserve(nCells))
	{
		if (!_theOctree)
			delete theOctree;
		delete cloud;
		return 0;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2];
	additionalParameters[0] = (void*)cloud;
	additionalParameters[1] = (void*)&resamplingMethod;

	//The process is so simple that MT is slower!
	//#ifdef ENABLE_MT_OCTREE
	//theOctree->executeFunctionForAllCellsAtLevel_MT(octreeLevel,
	//#else
	if (theOctree->executeFunctionForAllCellsAtLevel(octreeLevel,
													&resampleCellAtLevel,
													additionalParameters,
													progressCb,
													"Cloud Resampling") == 0)
	{
		//something went wrong
		delete cloud;
		cloud=0;

	}

	if (!_theOctree)
        delete theOctree;

	return cloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctree(GenericIndexedCloudPersist* theCloud, int newNumberOfPoints, SUBSAMPLING_CELL_METHOD subsamplingMethod, GenericProgressCallback* progressCb/*=0*/, DgmOctree* _theOctree/*=0*/)
{
	assert(theCloud);

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
			return 0;
	}

	//on cherche le niveau qui donne le nombre de points le plus proche de la consigne
	uchar bestLevel=theOctree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	ReferenceCloud* subsampledCloud = subsampleCloudWithOctreeAtLevel(theCloud,bestLevel,subsamplingMethod,progressCb,theOctree);

	if (!_theOctree)
		delete theOctree;

	return subsampledCloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud, uchar octreeLevel, SUBSAMPLING_CELL_METHOD subsamplingMethod, GenericProgressCallback* progressCb/*=0*/, DgmOctree* _theOctree/*=0*/)
{
	assert(theCloud);

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return 0;
		}
	}

	ReferenceCloud* cloud = new ReferenceCloud(theCloud);

	unsigned nCells = theOctree->getCellNumber(octreeLevel);
	if (!cloud->reserve(nCells))
	{
		if (!_theOctree)
			delete theOctree;
		delete cloud;
		return 0;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2];
	additionalParameters[0] = (void*)cloud;
	additionalParameters[1] = (void*)&subsamplingMethod;

	//The process is so simple that MT is slower!
	//#ifdef ENABLE_MT_OCTREE
	//theOctree->executeFunctionForAllCellsAtLevel_MT(octreeLevel,
	//#else
	if (theOctree->executeFunctionForAllCellsAtLevel(octreeLevel,
													&subsampleCellAtLevel,
													additionalParameters,
													progressCb,
													"Cloud Subsampling") == 0)
	{
		//something went wrong
		delete cloud;
		cloud=0;
	}

	if (!_theOctree)
        delete theOctree;

	return cloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudRandomly(GenericIndexedCloudPersist* theCloud, unsigned newNumberOfPoints, GenericProgressCallback* progressCb/*=0*/)
{
	assert(theCloud);

	unsigned theCloudSize = theCloud->size();

	//we put all input points in a ReferenceCloud
	ReferenceCloud* newCloud = new ReferenceCloud(theCloud);
	if (!newCloud->addPointIndex(0,theCloudSize))
	{
		delete newCloud;
		return 0;
	}

	//we have less points than requested?!
	if (theCloudSize <= newNumberOfPoints)
	{
		return newCloud;
	}

	unsigned pointsToRemove = theCloudSize-newNumberOfPoints;

	NormalizedProgress* normProgress=0;
    if (progressCb)
    {
        progressCb->setInfo("Random subsampling");
		normProgress = new NormalizedProgress(progressCb,pointsToRemove);
        progressCb->reset();
        progressCb->start();
    }

	//we randomly remove "theCloud.size() - newNumberOfPoints" points (much simpler)
	unsigned lastPointIndex = theCloudSize-1;
	for (unsigned i=0; i<pointsToRemove; ++i)
	{
		unsigned index = (unsigned)floor((float)rand()/(float)RAND_MAX * (float)lastPointIndex);
		newCloud->swap(index,lastPointIndex);
		--lastPointIndex;

		if (normProgress && !normProgress->oneStep())
		{
			//cancel process
			delete normProgress;
			delete newCloud;
			return 0;
		}
	}

	newCloud->resize(newNumberOfPoints); //always smaller, so it should be ok!

	if (normProgress)
		delete normProgress;

	return newCloud;
}

ReferenceCloud* CloudSamplingTools::resampleCloudSpatially(GenericIndexedCloudPersist* theCloud,
															float minDistance,
															DgmOctree* theOctree/*=0*/,
															GenericProgressCallback* progressCb/*=0*/)
{
	assert(theCloud);
    unsigned cloudSize = theCloud->size();

    DgmOctree *_theOctree=theOctree;
	if (!_theOctree)
	{
		_theOctree = new DgmOctree(theCloud);
		if (_theOctree->build()<(int)cloudSize)
		{
			delete _theOctree;
			return 0;
		}
	}

    ReferenceCloud* sampledCloud = new ReferenceCloud(theCloud);
    if (!sampledCloud->reserve(cloudSize))
	{
		if (!theOctree)
			delete _theOctree;
		return 0;
	}

	GenericChunkedArray<1,bool>* markers = new GenericChunkedArray<1,bool>(); //DGM: upgraded from vector, as this can be quite huge!
    if (!markers->resize(cloudSize,true,true))
	{
		markers->release();
		if (!theOctree)
			delete _theOctree;
		delete sampledCloud;
		return 0;
	}

	NormalizedProgress* normProgress=0;
    if (progressCb)
    {
        progressCb->setInfo("Spatial resampling");
		normProgress = new NormalizedProgress(progressCb,cloudSize);
        progressCb->reset();
        progressCb->start();
    }

	//for each point in the cloud that is still 'marked', we look
	//for its neighbors and remove their own marks
    DgmOctree::NearestNeighboursSphericalSearchStruct nss;
    nss.level = _theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(minDistance);
	
	markers->placeIteratorAtBegining();
    for (unsigned i=0; i<cloudSize; i++, markers->forwardIterator())
    {
		//progress indicator
		if (normProgress && !normProgress->oneStep())
		{
			//cancel process
			delete sampledCloud;
			sampledCloud = 0;
			break;
		}

		//no mark? we skip this point
		if (!markers->getCurrentValue())
            continue;

		//init neighbor search structure
		theCloud->getPoint(i,nss.queryPoint);
		bool inbounds = false;
		_theOctree->getTheCellPosWhichIncludesThePoint(&nss.queryPoint, nss.cellPos, nss.level, inbounds);
		nss.truncatedCellCode = (inbounds ? _theOctree->generateTruncatedCellCode(nss.cellPos, nss.level) : DgmOctree::INVALID_CELL_CODE);
		_theOctree->computeCellCenter(nss.cellPos, nss.level, nss.cellCenter);

        //add the points that lie in the same cell (faster)
		{
			ReferenceCloud* Y = _theOctree->getPointsInCell(nss.truncatedCellCode, nss.level, true);
			unsigned count = Y->size();
			try
			{
				nss.pointsInNeighbourhood.resize(count);
			}
			catch (std::bad_alloc) //out of memory
			{
				//stop process
				delete sampledCloud;
				sampledCloud = 0;
				break;
			}

			unsigned realCount = 0;
			DgmOctree::NeighboursSet::iterator it = nss.pointsInNeighbourhood.begin();
			for (unsigned j=0; j<count; ++j)
			{
				unsigned index = Y->getPointGlobalIndex(j);
				if (index != i && markers->getValue(index)) //no need to add the point itself and those already flagged off
				{
					it->point = Y->getPointPersistentPtr(j);
					it->pointIndex = index;
					++it;
					++realCount;
				}
			}
			nss.pointsInNeighbourhood.resize(realCount); //should be ok as realCount<=count
			nss.alreadyVisitedNeighbourhoodSize = 1;
		}

#ifdef TEST_CELLS_FOR_SPHERICAL_NN
		nss.pointsInSphericalNeighbourhood.clear();
#endif
		nss.prepare(minDistance,_theOctree->getCellSize(nss.level));
        
		//look for neighbors and 'de-mark' them
		{
			unsigned nbNeighbors = _theOctree->findNeighborsInASphereStartingFromCell(nss, minDistance, false);
			DgmOctree::NeighboursSet::iterator it = nss.pointsInNeighbourhood.begin();
			for (unsigned j=0; j<nbNeighbors; ++j, ++it)
				if (it->pointIndex != i)
					markers->setValue(it->pointIndex,false);
		}

        //At this stage, the ith point is the only one marked in a radius of <minDistance>.
        //Therefore it will necessarily be in the final cloud!
        if (!sampledCloud->addPointIndex(i))	//not enough memory
		{
			//stop process
			delete sampledCloud;
			sampledCloud = 0;
			break;
		}
    }

    if(normProgress)
	{
		delete normProgress;
		normProgress = 0;
	}

	if (!theOctree)
		delete _theOctree;

	markers->release();

    return sampledCloud;
}

bool CloudSamplingTools::resampleCellAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	SimpleCloud* cloud						= (SimpleCloud*)additionalParameters[0];
	RESAMPLING_CELL_METHOD resamplingMethod	= *((RESAMPLING_CELL_METHOD*)additionalParameters[1]);

	if (resamplingMethod == CELL_GRAVITY_CENTER)
	{
		const CCVector3* P = Neighbourhood(cell.points).getGravityCenter();
		if (P)
			cloud->addPoint(*P);
	}
	else //if (resamplingMethod == CELL_CENTER)
	{
		PointCoordinateType center[3];
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);
        cloud->addPoint(center);
	}

	return true;
}

bool CloudSamplingTools::subsampleCellAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	ReferenceCloud* cloud					    = (ReferenceCloud*)additionalParameters[0];
	SUBSAMPLING_CELL_METHOD subsamplingMethod	= *((SUBSAMPLING_CELL_METHOD*)additionalParameters[1]);

	unsigned selectedPointIndex=0;
	unsigned pointsCount = cell.points->size();

	if (subsamplingMethod == RANDOM_POINT)
	{
	    selectedPointIndex = ((unsigned)rand()) % pointsCount;
	}
	else // if (subsamplingMethod == NEAREST_POINT_TO_CELL_CENTER)
	{
		PointCoordinateType center[3];
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);

		ScalarType dist,minDist;
		minDist = CCVector3::vdistance2(cell.points->getPoint(0)->u,center);

		for (unsigned i=1;i<pointsCount;++i)
		{
			dist = CCVector3::vdistance2(cell.points->getPoint(i)->u,center);
			if (dist<minDist)
			{
				selectedPointIndex = i;
                minDist=dist;
            }
        }
    }

	return cloud->addPointIndex(cell.points->getPointGlobalIndex(selectedPointIndex));
}
