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

#include "GenericChunkedArray.h"
#include "AutoSegmentationTools.h"

//local
#include "GenericIndexedCloudPersist.h"
#include "GenericProgressCallback.h"
#include "ReferenceCloud.h"
#include "DgmOctree.h"
#include "FastMarchingForPropagation.h"
#include "ScalarFieldTools.h"
#include "ScalarField.h"

//system
#include <assert.h>

using namespace CCLib;

int AutoSegmentationTools::labelConnectedComponents(GenericIndexedCloudPersist* theCloud, uchar level, bool sixConnexity, GenericProgressCallback* progressCb, DgmOctree* _theOctree)
{
	if (!theCloud)
		return -1;

	//compute octree if none was provided
	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -1;
		}
	}

	//we use the default scalar field to store components labels
	theCloud->enableScalarField();

	int result = theOctree->extractCCs(level,sixConnexity,progressCb);

	//remove octree if it was not provided as input
	if (!_theOctree)
		delete theOctree;

	return result;
}

bool AutoSegmentationTools::extractConnectedComponents(GenericIndexedCloudPersist* theCloud, ReferenceCloudContainer& cc)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
		return false;

	//components should have already been labeled and labels should have been stored in the active scalar field!
	if (!theCloud->isScalarFieldEnabled())
		return false;

	//empty the input vector if necessary
	while (!cc.empty())
	{
		delete cc.back();
		cc.pop_back();
	}

	for (unsigned i=0; i<numberOfPoints; ++i)
	{
		ScalarType slabel = theCloud->getPointScalarValue(i);
		if (slabel >= 1) //labels start from 1! (this test rejects NaN values as well)
		{
			int ccLabel = static_cast<int>(theCloud->getPointScalarValue(i))-1; 

			//we fill the components vector with empty components until we reach the current label
			//(they will be "used" later)
			try
			{
				while (static_cast<size_t>(ccLabel) >= cc.size())
					cc.push_back(new ReferenceCloud(theCloud));
			}
			catch(std::bad_alloc)
			{
				//not enough memory
				cc.clear();
				return false;
			}

			//add the point to the current component
			if (!cc[ccLabel]->addPointIndex(i))
			{
				//not enough memory
				while (!cc.empty())
				{
					delete cc.back();
					cc.pop_back();
				}
				return false;
			}
		}
	}

	return true;
}

bool AutoSegmentationTools::frontPropagationBasedSegmentation(GenericIndexedCloudPersist* theCloud,
                                                                ScalarType minSeedDist,
                                                                uchar octreeLevel,
                                                                ReferenceCloudContainer& theSegmentedLists,
                                                                GenericProgressCallback* progressCb,
                                                                DgmOctree* _theOctree,
                                                                bool applyGaussianFilter,
                                                                float alpha)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
        return false;

	//compute octree if none was provided
	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return false;
		}
	}

	//on calcule le gradient (va ecraser le champ des distances)
	if (ScalarFieldTools::computeScalarFieldGradient(theCloud,true,true,progressCb,theOctree) < 0)
	{
		if (!_theOctree)
			delete theOctree;
		return false;
	}

	//et on lisse le resultat
	if (applyGaussianFilter)
	{
		uchar level = theOctree->findBestLevelForAGivenPopulationPerCell(NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION);
		PointCoordinateType cellSize = theOctree->getCellSize(level);
        ScalarFieldTools::applyScalarFieldGaussianFilter(static_cast<float>(cellSize/3),theCloud,-1,progressCb,theOctree);
	}

	unsigned seedPoints = 0;
	unsigned numberOfSegmentedLists = 0;

	//on va faire la propagation avec le FastMarching();
	FastMarchingForPropagation* fm = new FastMarchingForPropagation();

	fm->setJumpCoef(50.0);
	fm->setDetectionThreshold(alpha);

	int result = fm->init(theCloud,theOctree,octreeLevel);
	int octreeLength = OCTREE_LENGTH(octreeLevel)-1;

	if (result<0)
	{
		if (!_theOctree)
            delete theOctree;
		delete fm;
		return false;
	}

	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("FM Propagation");
		char buffer[256];
		sprintf(buffer,"Octree level: %i\nNumber of points: %u",octreeLevel,numberOfPoints);
		progressCb->setInfo(buffer);
		progressCb->start();
	}

	ScalarField* theDists = new ScalarField("distances");
	{
		ScalarType d = theCloud->getPointScalarValue(0);
		if (!theDists->resize(numberOfPoints,true,d))
		{
			if (!_theOctree)
				delete theOctree;
			return false;

		}
	}

	unsigned maxDistIndex = 0, begin = 0;
	CCVector3 startPoint;

	while (true)
	{
		ScalarType maxDist = NAN_VALUE;

		//on cherche la premiere distance superieure ou egale a "minSeedDist"
		while (begin<numberOfPoints)
		{
			const CCVector3 *thePoint = theCloud->getPoint(begin);
			const ScalarType& theDistance = theDists->getValue(begin);
			++begin;

			//FIXME DGM: what happens if SF is negative?!
			if (theCloud->getPointScalarValue(begin) >= 0 && theDistance >= minSeedDist)
			{
				maxDist = theDistance;
				startPoint = *thePoint;
				maxDistIndex = begin;
				break;
			}
		}

		//il n'y a plus de point avec des distances suffisamment grandes !
		if (maxDist<minSeedDist)
            break;

		//on finit la recherche du max
		for (unsigned i=begin;i<numberOfPoints;++i)
		{
			const CCVector3 *thePoint = theCloud->getPoint(i);
			const ScalarType& theDistance = theDists->getValue(i);

			if ((theCloud->getPointScalarValue(i)>=0.0)&&(theDistance > maxDist))
			{
				maxDist = theDistance;
				startPoint = *thePoint;
				maxDistIndex = i;
			}
		}

		//on lance la propagation a partir du point de distance maximale
		//propagateFromPoint(aList,_GradientNorms,maxDistIndex,octreeLevel,_gui);

		int pos[3];
		theOctree->getTheCellPosWhichIncludesThePoint(&startPoint,pos,octreeLevel);
		//clipping (important !)
		pos[0] = std::min(octreeLength,pos[0]);
		pos[1] = std::min(octreeLength,pos[1]);
		pos[2] = std::min(octreeLength,pos[2]);
		fm->setSeedCell(pos);
		++seedPoints;

		int result = fm->propagate();

		//si la propagation s'est bien passee
		if (result>=0)
		{
			//on la termine (i.e. on extrait les points correspondant)
			ReferenceCloud* newCloud = fm->extractPropagatedPoints();

			//si la liste convient
			//on la rajoute au groupe des listes segmentees
			theSegmentedLists.push_back(newCloud);
			++numberOfSegmentedLists;

			if (progressCb)
                progressCb->update(float(numberOfSegmentedLists % 100));

			fm->cleanLastPropagation();

			//break;
		}

		if (maxDistIndex == begin)
            ++begin;
	}

	if (progressCb)
		progressCb->stop();

	for (unsigned i=0;i<numberOfPoints;++i)
		theCloud->setPointScalarValue(i,theDists->getValue(i));

	delete fm;
	theDists->release();
	theDists=0;

	if (!_theOctree)
		delete theOctree;

	return true;
}
