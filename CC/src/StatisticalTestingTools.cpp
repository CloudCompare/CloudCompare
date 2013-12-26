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

#include "StatisticalTestingTools.h"

//local
#include "ReferenceCloud.h"
#include "DgmOctreeReferenceCloud.h"
#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "GenericDistribution.h"
#include "DgmOctree.h"
#include "GenericProgressCallback.h"
#include "Chi2Helper.h"
#include "ScalarField.h"
#include "NormalDistribution.h"

//system
#include <string.h>
#include <assert.h>
#include <list>

using namespace CCLib;

//! Max computable Chi2 distance
static double CHI2_MAX = 1e7;

//! An element of a double-chained-list structure (used by computeAdaptativeChi2Dist)
struct Chi2Class
{
	
	double pi;	/**< Probability Pi **/
	int n;		/**< Number of elements for the class **/

	//! Default constructor
	Chi2Class() : pi(0.0) , n(0) {}
	//! Constructor from parameters
	Chi2Class(double _pi, int _n) : pi(_pi) , n(_n) {}

};

//! An ordered list of Chi2 classes
typedef std::list<Chi2Class> Chi2ClassList;

double StatisticalTestingTools::computeAdaptativeChi2Dist(	const GenericDistribution* distrib,
															const GenericCloud* cloud,
															unsigned numberOfClasses,
															unsigned &finalNumberOfClasses,
															bool noClassCompression/*=false*/,
															ScalarType* histoMin/*=0*/,
															ScalarType* histoMax/*=0*/,
															unsigned* histoValues/*=0*/,
															double* npis/*=0*/)
{
    assert(distrib && cloud);
	unsigned n = cloud->size();

	if (n==0 || !distrib->isValid())
		return -1.0;

	//compute min and max (valid) values
	ScalarType minV=0,maxV=0;
	unsigned numberOfElements=0;
	{
		bool firstValidValue=true;
		for (unsigned i=0; i<n; ++i)
		{
			ScalarType V = cloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
				if (firstValidValue)
				{
					minV = maxV = V;
					firstValidValue = false;
				}
				else
				{
					if (V > maxV)
						maxV = V;
					else if (V < minV)
						minV = V;
				}
				++numberOfElements;
			}
		}
	}

	if (numberOfElements == 0)
        return -1.0;

    if (histoMin)
        minV = *histoMin;
    if (histoMax)
        maxV = *histoMax;

	//shall we automatically compute the number of classes?
	if (numberOfClasses==0)
	{
        numberOfClasses = (unsigned)ceil(sqrt((double)numberOfElements));
	}
	if (numberOfClasses<2)
	{
        return -2.0; //not enough points/classes
	}

	//try to allocate the histogram values array (if necessary)
	unsigned* histo = (histoValues ? histoValues : new unsigned[numberOfClasses]);
	if (!histo)
	{
		//not enough memory
		return -1.0;
	}
	memset(histo,0,sizeof(unsigned)*numberOfClasses);

	//accumulate histogram
	ScalarType dV = maxV-minV;
	unsigned histoBefore = 0;
	unsigned histoAfter = 0;
	if (dV > ZERO_TOLERANCE)
	{
		for (unsigned i=0;i<n;++i)
		{
			ScalarType V = cloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
				int bin = (int)floor((V-minV)*(ScalarType)numberOfClasses/dV);
				if (bin < 0)
				{
					histoBefore++;
				}
				else if (bin >= (int)numberOfClasses)
				{
					if (V > maxV)
						histoAfter++;
					else
						histo[numberOfClasses-1]++;
				}
				else
				{
					histo[bin]++;
				}
			}
		}
	}
	else
	{
		histo[0] = n;
	}

	//we build up the list of classes
	Chi2ClassList classes;
	//before?
	{
		if (histoBefore)
		{
			try
			{
				classes.push_back(Chi2Class(1.0e-6,(int)histoBefore));
			}
			catch(std::bad_alloc)
			{
				//not enough memory!
				return -1.0;
			}
		}
		double p1 = distrib->computePfromZero(minV);
		for (unsigned k=1; k<=numberOfClasses; ++k)
		{
			double p2 = distrib->computePfromZero(minV + (ScalarType)k * dV / (ScalarType)numberOfClasses);

			//add the class to the chain
			Chi2Class currentClass;
			currentClass.n = histo[k-1];
			currentClass.pi = p2-p1;
			if (npis)
				npis[k-1]= currentClass.pi * (double)numberOfElements;

			try
			{
				classes.push_back(currentClass);
			}
			catch(std::bad_alloc)
			{
				//not enough memory!
				return -1.0;
			}

			p1 = p2; //next intervale
		}
		if (histoAfter)
		{
			try
			{
				classes.push_back(Chi2Class(1.0e-6,(int)histoAfter));
			}
			catch(std::bad_alloc)
			{
				//not enough memory!
				return -1.0;
			}
		}
	}

	//classes compression
	if (!noClassCompression)
	{
		//lowest acceptable value: "K/n" (K=5 generally, but it could be 3 or 1 at the tail!)
		double minPi = 5.0/static_cast<double>(numberOfElements);

		while (classes.size() > 2)
		{
			//we look for the smallest class (smallest "npi")
			Chi2ClassList::iterator it = classes.begin();
			Chi2ClassList::iterator minIt = it;
			for (; it != classes.end(); ++it)
				if (it->pi < minIt->pi)
					minIt = it;

			if (minIt->pi >= minPi) //all classes are bigger than the minimum requirement
				break;

			//otherwise we must merge the smallest class with its neighbor (to make the classes repartition more equilibrated)
			Chi2ClassList::iterator smallestIt;
			{
				Chi2ClassList::iterator nextIt = minIt; nextIt++;
				if (minIt == classes.begin())
				{
					smallestIt = nextIt;
				}
				else
				{
					Chi2ClassList::iterator predIt = minIt; predIt--;
					smallestIt = (nextIt != classes.end() && nextIt->pi < predIt->pi ? nextIt : predIt);
				}
			}

			smallestIt->pi += minIt->pi;
			smallestIt->n += minIt->n;

			//we can remove the current class
			classes.erase(minIt);
		}
	}

	//we compute the Chi2 distance with the remaining classes
	double D2 = 0.0;
	{
		for (Chi2ClassList::iterator it = classes.begin(); it != classes.end(); ++it)
		{
			double npi = it->pi * (double)numberOfElements;
			if (npi != 0.0)
			{
				double temp = (double)it->n - npi;
				D2 += temp*(temp/npi);
				if (D2 >= CHI2_MAX)
				{
					D2 = CHI2_MAX;
					break;
				}
			}
			else
			{
				D2 = CHI2_MAX;
				break;
			}
		}
	}

	if (!histoValues)
        delete[] histo;

	finalNumberOfClasses = (unsigned)classes.size();

	return D2;
}

double StatisticalTestingTools::computeChi2Fractile(double p, int d)
{
	return Chi2Helper::critchi(p,d);
}

double StatisticalTestingTools::computeChi2Probability(double chi2result, int d)
{
	return Chi2Helper::pochisq(chi2result,d);
}

double StatisticalTestingTools::testCloudWithStatisticalModel(const GenericDistribution* distrib,
                                                              GenericIndexedCloudPersist* theCloud,
                                                              unsigned numberOfNeighbours,
                                                              double pTrust,
                                                              GenericProgressCallback* progressCb/*=0*/,
                                                              DgmOctree* _theOctree/*=0*/)
{
	assert(theCloud);

	if (!distrib->isValid())
		return -1.0;

	DgmOctree* theOctree = _theOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -2.0;
		}
	}

	//on active le champ scalaire (IN) pour recevoir les distances du Chi2
	theCloud->enableScalarField();

	uchar level = theOctree->findBestLevelForAGivenPopulationPerCell(numberOfNeighbours);

	unsigned numberOfChi2Classes = (unsigned)ceil(sqrt((double)numberOfNeighbours));

	//Chi2 hisogram values
	unsigned* histoValues = new unsigned[numberOfChi2Classes];
	if (!histoValues)
	{
		if (!_theOctree)
			delete theOctree;
		return -3.0;
	}

	ScalarType* histoMin = 0, customHistoMin = 0;
	ScalarType* histoMax = 0, customHistoMax = 0;
	if (strcmp(distrib->getName(),"Gauss")==0)
	{
		const NormalDistribution* nDist = static_cast<const NormalDistribution*>(distrib);
		ScalarType mu=0, sigma2=0;
		nDist->getParameters(mu, sigma2);
		customHistoMin = mu - (ScalarType)3.0 * sqrt(sigma2);
		histoMin = &customHistoMin;
		customHistoMax = mu + (ScalarType)3.0 * sqrt(sigma2);
		histoMax = &customHistoMax;
	}
	else if (strcmp(distrib->getName(),"Weibull")==0)
	{
		customHistoMin = 0;
		histoMin = &customHistoMin;
	}

	//additionnal parameters for local process
	void* additionalParameters[] = {	(void*)distrib,
										(void*)&numberOfNeighbours,
										(void*)&numberOfChi2Classes,
										(void*)histoValues,
										(void*)histoMin,
										(void*)histoMax};

	double maxChi2 = -1.0;

	//let's compute Chi2 distances
#ifndef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtStartingLevel(level,
#else
	if (theOctree->executeFunctionForAllCellsAtStartingLevel_MT(level,
#endif
															&computeLocalChi2DistAtLevel,
															additionalParameters,
															numberOfNeighbours/2,
															numberOfNeighbours*3,
															progressCb,
															"Statistical Test")>0) //sucess
	{
		if (!progressCb || !progressCb->isCancelRequested())
		{
			//theoretical Chi2 fractile
			maxChi2 = computeChi2Fractile(pTrust, numberOfChi2Classes-1);
			maxChi2 = sqrt(maxChi2); //on travaille avec les racines carrees des distances du Chi2
		}
	}

	delete[] histoValues;
	histoValues=0;

	if (!_theOctree)
        delete theOctree;

	return maxChi2;
}

bool StatisticalTestingTools::computeLocalChi2DistAtLevel(	const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress/*=0*/)
{
	//variables additionnelles
	GenericDistribution* statModel		= (GenericDistribution*)additionalParameters[0];
	unsigned numberOfNeighbours         = *(unsigned*)additionalParameters[1];
	unsigned numberOfChi2Classes		= *(unsigned*)additionalParameters[2];
	unsigned* histoValues				= (unsigned*)additionalParameters[3];
	ScalarType* histoMin				= (ScalarType*)additionalParameters[4];
	ScalarType* histoMax				= (ScalarType*)additionalParameters[5];

	//number of points in the current cell
	unsigned n = cell.points->size();

	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level												= cell.level;
	nNSS.minNumberOfNeighbors								= numberOfNeighbours;
	nNSS.truncatedCellCode									= cell.truncatedCode;
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//we already know the points of the first cell (this is the one we are currently processing!)
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (std::bad_alloc) //out of memory
		{
			return false;
		}

		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j=0;j<n;++j,++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	ReferenceCloud neighboursCloud(cell.points->getAssociatedCloud());
	if (!neighboursCloud.reserve(numberOfNeighbours))
	{
		//not enough memory!
		return false;
	}

	for (unsigned i=0;i<n;++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);
		ScalarType D = cell.points->getPointScalarValue(i);

		if (ScalarField::ValidValue(D))
		{
			//nNSS.theNearestPoints.clear();

			unsigned k = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS,true);
			if (k>numberOfNeighbours)
				k=numberOfNeighbours;

			neighboursCloud.clear(false);
			for (unsigned j=0; j<k; ++j)
				neighboursCloud.addPointIndex(nNSS.pointsInNeighbourhood[j].pointIndex);

			unsigned finalNumberOfChi2Classes=0;
			//VERSION "SYMPA" (test grossier)
			double Chi2Dist = (ScalarType)computeAdaptativeChi2Dist(statModel,&neighboursCloud,numberOfChi2Classes,finalNumberOfChi2Classes,true,histoMin,histoMax,histoValues);
			//VERSION "SEVERE" (test ultra-precis)
			//double Chi2Dist = (ScalarType)computeAdaptativeChi2Dist(statModel,&neighboursCloud,numberOfChi2Classes,finalNumberOfChi2Classes,false,histoMin,histoMax,histoValues);

			D = (Chi2Dist >= 0.0 ? (ScalarType)sqrt(Chi2Dist) : NAN_VALUE);
		}

		//We assume that "IN" and "OUT" scalar fields are different!
		cell.points->setPointScalarValue(i,D);

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}
