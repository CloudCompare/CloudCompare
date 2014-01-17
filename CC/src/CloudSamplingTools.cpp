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

GenericIndexedCloud* CloudSamplingTools::resampleCloudWithOctree(	GenericIndexedCloudPersist* theCloud,
																	int newNumberOfPoints,
																	RESAMPLING_CELL_METHOD resamplingMethod,
																	GenericProgressCallback* progressCb,
																	DgmOctree* inputOctree)
{
	assert(theCloud);

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
			return 0;
	}

	//on cherche le niveau qui donne le nombre de points le plus proche de la consigne
	uchar bestLevel=theOctree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	GenericIndexedCloud* sampledCloud = resampleCloudWithOctreeAtLevel(theCloud,bestLevel,resamplingMethod,progressCb,theOctree);

	if (!inputOctree)
		delete theOctree;

	return sampledCloud;
}

SimpleCloud* CloudSamplingTools::resampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud, uchar octreeLevel, RESAMPLING_CELL_METHOD resamplingMethod, GenericProgressCallback* progressCb, DgmOctree* inputOctree)
{
	assert(theCloud);

	DgmOctree* theOctree = inputOctree;
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
		if (!inputOctree)
			delete theOctree;
		delete cloud;
		return 0;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2] = {	(void*)cloud,
										(void*)&resamplingMethod };

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

	if (!inputOctree)
        delete theOctree;

	return cloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctree(	GenericIndexedCloudPersist* theCloud,
																int newNumberOfPoints,
																SUBSAMPLING_CELL_METHOD subsamplingMethod,
																GenericProgressCallback* progressCb/*=0*/,
																DgmOctree* inputOctree/*=0*/)
{
	assert(theCloud);

	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
			return 0;
	}

	//on cherche le niveau qui donne le nombre de points le plus proche de la consigne
	uchar bestLevel=theOctree->findBestLevelForAGivenCellNumber(newNumberOfPoints);

	ReferenceCloud* subsampledCloud = subsampleCloudWithOctreeAtLevel(theCloud,bestLevel,subsamplingMethod,progressCb,theOctree);

	if (!inputOctree)
		delete theOctree;

	return subsampledCloud;
}

ReferenceCloud* CloudSamplingTools::subsampleCloudWithOctreeAtLevel(GenericIndexedCloudPersist* theCloud,
																	uchar octreeLevel,
																	SUBSAMPLING_CELL_METHOD subsamplingMethod,
																	GenericProgressCallback* progressCb/*=0*/,
																	DgmOctree* inputOctree/*=0*/)
{
	assert(theCloud);

	DgmOctree* theOctree = inputOctree;
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
		if (!inputOctree)
			delete theOctree;
		delete cloud;
		return 0;
	}

	//structure contenant les parametres additionnels
	void* additionalParameters[2] = {	(void*)cloud,
										(void*)&subsamplingMethod };

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

	if (!inputOctree)
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
															PointCoordinateType minDistance,
															DgmOctree* inputOctree/*=0*/,
															GenericProgressCallback* progressCb/*=0*/)
{
	assert(theCloud);
    unsigned cloudSize = theCloud->size();

    DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build() < static_cast<int>(cloudSize))
		{
			delete theOctree;
			return 0;
		}
	}
	assert(theOctree && theOctree->associatedCloud() == theCloud);

	//output cloud
    ReferenceCloud* sampledCloud = new ReferenceCloud(theCloud);
	const unsigned c_reserveStep = 65536;
    if (!sampledCloud->reserve(std::min(cloudSize,c_reserveStep)))
	{
		if (!inputOctree)
			delete theOctree;
		return 0;
	}

	GenericChunkedArray<1,bool>* markers = new GenericChunkedArray<1,bool>(); //DGM: upgraded from vector, as this can be quite huge!
    if (!markers->resize(cloudSize,true,true))
	{
		markers->release();
		if (!inputOctree)
			delete theOctree;
		delete sampledCloud;
		return 0;
	}

	//progress notification
	NormalizedProgress* normProgress = 0;
    if (progressCb)
    {
		progressCb->setMethodTitle("Spatial resampling");
		char buffer[256];
		sprintf(buffer,"Points: %i\nMin dist.: %f",cloudSize,minDistance);
        progressCb->setInfo(buffer);
		normProgress = new NormalizedProgress(progressCb,cloudSize);
        progressCb->reset();
        progressCb->start();
    }

    unsigned char level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(minDistance);

	//for each point in the cloud that is still 'marked', we look
	//for its neighbors and remove their own marks
	markers->placeIteratorAtBegining();
	bool error = false;
    for (unsigned i=0; i<cloudSize; i++, markers->forwardIterator())
    {
		//no mark? we skip this point
		if (markers->getCurrentValue())
		{
			//init neighbor search structure
			const CCVector3* P = theCloud->getPoint(i);

			//look for neighbors and 'de-mark' them
			{
				DgmOctree::NeighboursSet neighbours;
				theOctree->getPointsInSphericalNeighbourhood(*P,minDistance,neighbours,level);
				for (DgmOctree::NeighboursSet::iterator it = neighbours.begin(); it != neighbours.end(); ++it)
					if (it->pointIndex != i)
						markers->setValue(it->pointIndex,false);
			}

			//At this stage, the ith point is the only one marked in a radius of <minDistance>.
			//Therefore it will necessarily be in the final cloud!
			if (sampledCloud->size() == sampledCloud->capacity() && !sampledCloud->reserve(sampledCloud->capacity()+c_reserveStep))
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
		if (normProgress && !normProgress->oneStep())
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
		sampledCloud = 0;
	}

    if(normProgress)
	{
		delete normProgress;
		normProgress = 0;
		progressCb->stop();
	}

	if (!inputOctree)
		delete theOctree;

	markers->release();

    return sampledCloud;
}

bool CloudSamplingTools::resampleCellAtLevel(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress/*=0*/)
{
	SimpleCloud* cloud						= static_cast<SimpleCloud*>(additionalParameters[0]);
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
		PointCoordinateType center[3];
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);
        cloud->addPoint(center);
	}

	if (nProgress && !nProgress->steps(cell.points->size()))
		return false;

	return true;
}

bool CloudSamplingTools::subsampleCellAtLevel(	const DgmOctree::octreeCell& cell,
												void** additionalParameters,
												NormalizedProgress* nProgress/*=0*/)
{
	ReferenceCloud* cloud					    = static_cast<ReferenceCloud*>(additionalParameters[0]);
	SUBSAMPLING_CELL_METHOD subsamplingMethod	= *static_cast<SUBSAMPLING_CELL_METHOD*>(additionalParameters[1]);

	unsigned selectedPointIndex = 0;
	unsigned pointsCount = cell.points->size();

	if (subsamplingMethod == RANDOM_POINT)
	{
	    selectedPointIndex = (static_cast<unsigned>(rand()) % pointsCount);

		if (nProgress && !nProgress->steps(pointsCount))
			return false;
	}
	else // if (subsamplingMethod == NEAREST_POINT_TO_CELL_CENTER)
	{
		PointCoordinateType center[3];
		cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,center,true);

		PointCoordinateType minDist = CCVector3::vdistance2(cell.points->getPoint(0)->u,center);

		for (unsigned i=1; i<pointsCount; ++i)
		{
			PointCoordinateType dist = CCVector3::vdistance2(cell.points->getPoint(i)->u,center);
			if (dist < minDist)
			{
				selectedPointIndex = i;
                minDist = dist;
            }

			if (nProgress && !nProgress->oneStep())
				return false;
        }
    }

	return cloud->addPointIndex(cell.points->getPointGlobalIndex(selectedPointIndex));
}
