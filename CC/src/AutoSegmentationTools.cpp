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

#include <AutoSegmentationTools.h>

//local
#include <FastMarchingForPropagation.h>
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>

//System
#include <algorithm>

using namespace CCLib;

int AutoSegmentationTools::labelConnectedComponents(GenericIndexedCloudPersist* theCloud,
													unsigned char level,
													bool sixConnexity/*=false*/,
													GenericProgressCallback* progressCb/*=0*/,
													DgmOctree* inputOctree/*=0*/)
{
	if (!theCloud)
	{
		return -1;
	}

	//compute octree if none was provided
	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return -1;
		}
	}

	//we use the default scalar field to store components labels
	if (!theCloud->enableScalarField())
	{
		//failed to enable a scalar field
		return -1;
	}

	int result = theOctree->extractCCs(level, sixConnexity, progressCb);

	//remove octree if it was not provided as input
	if (theOctree && !inputOctree)
	{
		delete theOctree;
	}

	return result;
}

bool AutoSegmentationTools::extractConnectedComponents(GenericIndexedCloudPersist* theCloud, ReferenceCloudContainer& cc)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
	{
		return false;
	}

	//components should have already been labeled and labels should have been stored in the active scalar field!
	if (!theCloud->isScalarFieldEnabled())
	{
		return false;
	}

	//empty the input vector if necessary
	while (!cc.empty())
	{
		delete cc.back();
		cc.pop_back();
	}

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		ScalarType slabel = theCloud->getPointScalarValue(i);
		if (slabel >= 1) //labels start from 1! (this test rejects NaN values as well)
		{
			int ccLabel = static_cast<int>(theCloud->getPointScalarValue(i)) - 1;

			//we fill the components vector with empty components until we reach the current label
			//(they will be "used" later)
			try
			{
				while (static_cast<std::size_t>(ccLabel) >= cc.size())
				{
					cc.push_back(new ReferenceCloud(theCloud));
				}
			}
			catch (const std::bad_alloc&)
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

bool AutoSegmentationTools::frontPropagationBasedSegmentation(	GenericIndexedCloudPersist* theCloud,
																PointCoordinateType radius,
																ScalarType minSeedDist,
																unsigned char octreeLevel,
																ReferenceCloudContainer& theSegmentedLists,
																GenericProgressCallback* progressCb,
																DgmOctree* inputOctree,
																bool applyGaussianFilter,
																float alpha)
{
	unsigned numberOfPoints = (theCloud ? theCloud->size() : 0);
	if (numberOfPoints == 0)
	{
		return false;
	}

	//compute octree if none was provided
	DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb) < 1)
		{
			delete theOctree;
			return false;
		}
	}

	//on calcule le gradient (va ecraser le champ des distances)
	if (ScalarFieldTools::computeScalarFieldGradient(theCloud, radius, true, true, progressCb, theOctree) < 0)
	{
		if (!inputOctree)
			delete theOctree;
		return false;
	}

	//et on lisse le resultat
	if (applyGaussianFilter)
	{
		ScalarFieldTools::applyScalarFieldGaussianFilter(radius / 3, theCloud, -1, progressCb, theOctree);
	}

	unsigned seedPoints = 0;
	unsigned numberOfSegmentedLists = 0;

	//on va faire la propagation avec le FastMarching();
	FastMarchingForPropagation* fm = new FastMarchingForPropagation();

	fm->setJumpCoef(50.0);
	fm->setDetectionThreshold(alpha);

	int result = fm->init(theCloud, theOctree, octreeLevel);
	int octreeLength = DgmOctree::OCTREE_LENGTH(octreeLevel) - 1;

	if (result < 0)
	{
		if (!inputOctree)
		{
			delete theOctree;
		}
		delete fm;
		return false;
	}

	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("FM Propagation");
			char buffer[256];
			sprintf(buffer, "Octree level: %i\nNumber of points: %u", octreeLevel, numberOfPoints);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	ScalarField* theDists = new ScalarField("distances");
	{
		ScalarType d = theCloud->getPointScalarValue(0);
		if (!theDists->resizeSafe(numberOfPoints, true, d))
		{
			if (!inputOctree)
				delete theOctree;
			theDists->release();
			return false;
		}
	}

	unsigned maxDistIndex = 0;
	unsigned begin = 0;
	CCVector3 startPoint;

	while (true)
	{
		ScalarType maxDist = NAN_VALUE;

		//on cherche la premiere distance superieure ou egale a "minSeedDist"
		while (begin<numberOfPoints)
		{
			const CCVector3 *thePoint = theCloud->getPoint(begin);
			const ScalarType& theDistance = theDists->at(begin);
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
		if (maxDist < minSeedDist)
		{
			break;
		}

		//on finit la recherche du max
		for (unsigned i = begin; i<numberOfPoints; ++i)
		{
			const CCVector3 *thePoint = theCloud->getPoint(i);
			const ScalarType& theDistance = theDists->at(i);

			if ((theCloud->getPointScalarValue(i) >= 0.0) && (theDistance > maxDist))
			{
				maxDist = theDistance;
				startPoint = *thePoint;
				maxDistIndex = i;
			}
		}

		//set seed point
		{
			Tuple3i cellPos;
			theOctree->getTheCellPosWhichIncludesThePoint(&startPoint, cellPos, octreeLevel);
			//clipping (important!)
			cellPos.x = std::min(octreeLength, cellPos.x);
			cellPos.y = std::min(octreeLength, cellPos.y);
			cellPos.z = std::min(octreeLength, cellPos.z);
			fm->setSeedCell(cellPos);
			++seedPoints;
		}

		int result = fm->propagate();

		//if the propagation was successful
		if (result >= 0)
		{
			//we extract the corresponding points
			ReferenceCloud* newCloud = new ReferenceCloud(theCloud);
			
			if (fm->extractPropagatedPoints(newCloud) && newCloud->size() != 0)
			{
				theSegmentedLists.push_back(newCloud);
				++numberOfSegmentedLists;
			}
			else
			{
				//not enough memory?!
				delete newCloud;
				newCloud = nullptr;
			}

			if (progressCb)
			{
				progressCb->update(static_cast<float>(numberOfSegmentedLists % 100));
			}

			fm->cleanLastPropagation();

			//break;
		}

		if (maxDistIndex == begin)
		{
			++begin;
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	for (unsigned i = 0; i < numberOfPoints; ++i)
	{
		theCloud->setPointScalarValue(i, theDists->at(i));
	}

	theDists->release();
	theDists = nullptr;

	if (fm)
	{
		delete fm;
		fm = nullptr;
	}

	if (theOctree && !inputOctree)
	{
		delete theOctree;
		theOctree = nullptr;
	}

	return true;
}
