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

#include "ScalarFieldTools.h"

//local
#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "ReferenceCloud.h"
#include "GenericProgressCallback.h"
#include "GenericChunkedArray.h"
#include "ScalarField.h"

//system
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <vector>

using namespace CCLib;

void ScalarFieldTools::SetScalarValueToNaN(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = NAN_VALUE;
}

void ScalarFieldTools::SetScalarValueToZero(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = 0;
}

int ScalarFieldTools::computeScalarFieldGradient(GenericIndexedCloudPersist* theCloud, bool euclideanDistances, bool sameInAndOutScalarField, GenericProgressCallback* progressCb, DgmOctree* theCloudOctree)
{
	if (!theCloud)
        return -1;

	DgmOctree* theOctree = theCloudOctree;
	if (!theOctree)
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return -2;
		}
	}

	uchar octreeLevel = theOctree->findBestLevelForAGivenPopulationPerCell(NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION);

	ScalarField* theGradientNorms = new ScalarField("gradient norms");
	ScalarField* _theGradientNorms = 0;

	//mode champ scalaire "IN" et "OUT" identique
	if (sameInAndOutScalarField)
	{
		if (!theGradientNorms->reserve(theCloud->size())) //not enough memory
		{
			if (!theCloudOctree)
				delete theOctree;
			theGradientNorms->release();
			return -3;
		}
		_theGradientNorms = theGradientNorms;
	}
	else
	//mode champs scalaires "IN" et "OUT" dfferents (par defaut)
	{
		//on initialise les distances (IN - en ecriture) pour recevoir les normes du gradient
		if (!theCloud->enableScalarField())
		{
			if (!theCloudOctree)
				delete theOctree;
			theGradientNorms->release();
			return -4;
		}
	}

	//structure contenant les parametres additionnels
	PointCoordinateType radius = theOctree->getCellSize(octreeLevel);
	void* additionalParameters[3] = {	static_cast<void*>(&euclideanDistances),
										static_cast<void*>(&radius),
										static_cast<void*>(_theGradientNorms)
	};

	int result = 0;

#ifndef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtStartingLevel(octreeLevel,
#else
	if (theOctree->executeFunctionForAllCellsAtStartingLevel_MT(octreeLevel,
#endif
															computeMeanGradientOnPatch,
															additionalParameters,
															NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION/2,
															NUMBER_OF_POINTS_FOR_GRADIENT_COMPUTATION*3,
															progressCb,
															"Gradient Computation")==0)
	{
		//something went wrong
		result = -5;
	}

	if (!theCloudOctree)
        delete theOctree;

	theGradientNorms->release();
	theGradientNorms=0;

    return result;
}

bool ScalarFieldTools::computeMeanGradientOnPatch(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress/*=0*/)
{
	//variables additionnelles
	bool euclideanDistances			= *((bool*)additionalParameters[0]);
	PointCoordinateType radius		= *((PointCoordinateType*)additionalParameters[1]);
	ScalarField* theGradientNorms	= (ScalarField*)additionalParameters[2];

	//nombre de points dans la cellule courante
	unsigned n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//on connait deja les points de la premiere cellule
	//(c'est la cellule qu'on est en train de traiter !)
	{
		try
		{
			nNSS.pointsInNeighbourhood.resize(n);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}
		DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j=0; j<n; ++j,++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

	for (unsigned i=0; i<n; ++i)
	{
		ScalarType gN = NAN_VALUE;

		ScalarType d1 = cell.points->getPointScalarValue(i);

        if (ScalarField::ValidValue(d1))
		{
			 cell.points->getPoint(i,nNSS.queryPoint);

			//we extract the point's neighbors
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
			unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,true);

            //if more than one neighbour (the query point itself)
			if (k > 1)
			{
				CCVector3d sum(0,0,0);
				unsigned counter = 0;

				//j=1 because the first point is the query point itself --> contribution = 0
				for (unsigned j=1; j<k; ++j)
				{
					ScalarType d2 = cloud->getPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex);
					if (ScalarField::ValidValue(d2))
					{
						CCVector3 u = *nNSS.pointsInNeighbourhood[j].point - nNSS.queryPoint;
						PointCoordinateType norm2 = u.norm2();

						if (norm2 > ZERO_TOLERANCE)
						{
                            PointCoordinateType dd = static_cast<PointCoordinateType>(d2 - d1);
							if (!euclideanDistances || dd*dd < static_cast<PointCoordinateType>(1.01) * norm2)
							{
								dd /= norm2;
								sum.x += static_cast<double>(u.x * dd); //warning: and here 'dd'=dd/norm2 ;)
								sum.y += static_cast<double>(u.y * dd);
								sum.z += static_cast<double>(u.z * dd);
								++counter;
							}
						}
					}
				}

				if (counter != 0)
					gN = static_cast<ScalarType>(sum.norm()/counter);
			}
		}

		if (theGradientNorms)
			//if "IN" and "OUT" SFs are the same
			theGradientNorms->setValue(cell.points->getPointGlobalIndex(i),gN);
		else
			//if "IN" and "OUT" SFs are different
			cell.points->setPointScalarValue(i,gN);

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

bool ScalarFieldTools::applyScalarFieldGaussianFilter(PointCoordinateType sigma,
													  GenericIndexedCloudPersist* theCloud,
													  PointCoordinateType sigmaSF,
													  GenericProgressCallback* progressCb,
													  DgmOctree* theCloudOctree)
{
	if (!theCloud)
        return false;

	unsigned n = theCloud->size();
	if (n==0)
        return false;

	DgmOctree* theOctree = 0;
	if (theCloudOctree)
        theOctree = theCloudOctree;
	else
	{
		theOctree = new DgmOctree(theCloud);
		if (theOctree->build(progressCb)<1)
		{
			delete theOctree;
			return false;
		}
	}

    //best octree level
	uchar level = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(3.0f*sigma);

	//output scalar field should be different than input one
	theCloud->enableScalarField();

	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Gaussian filter");
		char infos[256];
		sprintf(infos,"Level: %i\n",level);
		progressCb->setInfo(infos);
	}

    void* additionalParameters[2] = {	additionalParameters[0] = (void*)&sigma,
										additionalParameters[1] = (void*)&sigmaSF
	};

	bool success = true;

#ifndef ENABLE_MT_OCTREE
	if (theOctree->executeFunctionForAllCellsAtLevel(level,
#else
	if (theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#endif
													computeCellGaussianFilter,
                                                    additionalParameters,
                                                    progressCb,
													"Gaussian Filter computation")==0)
	{
		//something went wrong
		success = false;
	}

	return success;
}

//FONCTION "CELLULAIRE" DE CALCUL DU FILTRE GAUSSIEN (PAR PROJECTION SUR LE PLAN AUX MOINDRES CARRES)
//DETAIL DES PARAMETRES ADDITIONNELS (2) :
// [0] -> (PointCoordinateType*) sigma : gauss function sigma
// [1] -> (PointCoordinateType*) sigmaSF : used when in "bilateral modality" - if -1 pure gaussian filtering is performed
bool ScalarFieldTools::computeCellGaussianFilter(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress/*=0*/)
{
	//variables additionnelles
	PointCoordinateType sigma	= *((PointCoordinateType*)additionalParameters[0]);
    PointCoordinateType sigmaSF	= *((PointCoordinateType*)additionalParameters[1]);

    //we use only the squared value of sigma
	PointCoordinateType sigma2 = 2*sigma*sigma;
	PointCoordinateType radius = 3*sigma; //2.5 sigma > 99%

	//we use only the squared value of sigmaSF
    PointCoordinateType sigmaSF2 = 2*sigmaSF*sigmaSF;

	//number of points inside the current cell
	unsigned n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//we already know the points lying in the first cell (this is the one we are treating :)
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}
	
	DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	{
		for (unsigned i=0; i<n; ++i,++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

    //Pure Gaussian Filtering
    if (sigmaSF == -1)
    {
        for (unsigned i=0; i<n; ++i) //for each point in cell
        {
            //we get the points inside a spherical neighbourhood (radius: '3*sigma')
            cell.points->getPoint(i,nNSS.queryPoint);
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
            unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

            //each point adds a contribution weighted by its distance to the sphere center
            it = nNSS.pointsInNeighbourhood.begin();
            double meanValue = 0.0;
            double wSum = 0.0;
            for (unsigned j=0;j<k;++j,++it)
            {
                double weight = exp(-(it->squareDistd)/sigma2); //PDF: -exp(-(x-mu)^2/(2*sigma^2))
                ScalarType val = cloud->getPointScalarValue(it->pointIndex);
                //scalar value must be valid
				if (ScalarField::ValidValue(val))
                {
                    meanValue += static_cast<double>(val) * weight;
                    wSum += weight;
                }
            }

			ScalarType newValue = (wSum > 0.0 ? static_cast<ScalarType>(meanValue / wSum) : NAN_VALUE);

            cell.points->setPointScalarValue(i,newValue);

			if (nProgress && !nProgress->oneStep())
				return false;
        }
    }
    //Bilateral Filtering using the second sigma parameters on values (when given)
    else
    {
        for (unsigned i=0;i<n;++i) //for each point in cell
        {
            ScalarType queryValue = cell.points->getPointScalarValue(i); //scalar of the query point

            //we get the points inside a spherical neighbourhood (radius: '3*sigma')
            cell.points->getPoint(i,nNSS.queryPoint);
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
            unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

            //each point adds a contribution weighted by its distance to the sphere center
            it = nNSS.pointsInNeighbourhood.begin();
            double meanValue = 0.0;
            double wSum = 0.0;
            for (unsigned j=0;j<k;++j,++it)
            {
                ScalarType val = cloud->getPointScalarValue(it->pointIndex);
				ScalarType dSF = queryValue - val;
                double weight = exp(-(it->squareDistd)/sigma2) * exp(-(dSF*dSF)/sigmaSF2);
                //scalar value must be valid
				if (ScalarField::ValidValue(val))
                {
                    meanValue += (double)val * weight;
                    wSum += weight;
                }
            }

            cell.points->setPointScalarValue(i,wSum > 0.0 ? (ScalarType)(meanValue / wSum) : NAN_VALUE);

			if (nProgress && !nProgress->oneStep())
				return false;
        }
    }

	return true;
}

void ScalarFieldTools::multiplyScalarFields(GenericIndexedCloud* firstCloud, GenericIndexedCloud* secondCloud, GenericProgressCallback* progressCb)
{
	if (!firstCloud || !secondCloud)
		return;

	unsigned n1 = firstCloud->size();
	if (n1 != secondCloud->size() || n1==0)
		return;

	for (unsigned i=0;i<n1;++i)
	{
		ScalarType V1 = firstCloud->getPointScalarValue(i);
		ScalarType V2 = secondCloud->getPointScalarValue(i);

		firstCloud->setPointScalarValue(i,ScalarField::ValidValue(V1) && ScalarField::ValidValue(V2) ? V1*V2 : NAN_VALUE);
	}
}

void ScalarFieldTools::computeScalarFieldExtremas(const GenericCloud* theCloud, ScalarType& minV, ScalarType& maxV)
{
    assert(theCloud);

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
        return;

	minV = maxV = 0;

	bool firstValue=true;

	for (unsigned i=0;i<numberOfPoints;++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
		    if (!firstValue)
		    {
                if (V<minV)
                    minV=V;
                else if (V>maxV)
                    maxV=V;
		    }
		    else
		    {
		        minV = maxV = V;
		        firstValue = false;
		    }
		}
	}
}

unsigned ScalarFieldTools::countScalarFieldValidValues(const GenericCloud* theCloud)
{
    assert(theCloud);

	unsigned count = 0;

	unsigned n = theCloud->size();
	for (unsigned i=0; i<n; ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
			++count;
	}

	return count;
}

void ScalarFieldTools::computeScalarFieldHistogram(const GenericCloud* theCloud, unsigned numberOfClasses, std::vector<int>& histo)
{
    assert(theCloud);

    histo.clear();

	if (numberOfClasses<2)
	{
	    histo.push_back(theCloud->size());
        return;
	}

	//reset
	try
	{
		histo.resize(numberOfClasses,0);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return;
	}

	//on calcule les extremas
	ScalarType minV,maxV;
	computeScalarFieldExtremas(theCloud,minV,maxV);

	//on en deduit le pas de l'historgramme
	ScalarType invStep = (maxV>minV ? (ScalarType)numberOfClasses / (maxV-minV) : 0.0f);

	//projection des valeurs dans l'histo
	for (unsigned i=0;i<theCloud->size();++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
            int aimClass = (int)((V-minV)*invStep);
            if (aimClass == (int)numberOfClasses)
                --aimClass; //attention a la frontiere sup.

            ++histo[aimClass];
		}
	}
}

bool ScalarFieldTools::computeKmeans(	const GenericCloud* theCloud,
										uchar K,
										KMeanClass kmcc[],
										GenericProgressCallback* progressCb)
{
	assert(theCloud);
	if (K == 0)
		return false;

	unsigned n = theCloud->size();
	if (n==0)
        return false;

	//on a besoin de memoire ici !
	std::vector<ScalarType> theKMeans;		//le centre des K clusters
	std::vector<uchar> belongings;			//l'appartenance d'un point a un cluster
	std::vector<ScalarType> minDistsToMean;	//les distances au centre de cluster le plus proche
	std::vector<ScalarType> theKSums;		//le cumuls de distance des k clusters
	std::vector<unsigned> theKNums;			//le nombre de point par cluster
	std::vector<unsigned> theOldKNums;		//le nombre de point par cluster (ancien)

	try
	{
		theKMeans.resize(n);
		belongings.resize(n);
		minDistsToMean.resize(n);
		theKSums.resize(K);
		theKNums.resize(K);
		theOldKNums.resize(K);
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	//on recupere les extremas
	ScalarType minV,maxV;
	computeScalarFieldExtremas(theCloud, minV, maxV);

	//init classes centers (regularly sampled)
	{
		ScalarType step = (maxV - minV) / ScalarType(K);
		for (uchar j=0;j<K;++j)
			theKMeans[j] = minV + ScalarType(j)*step;
	}

	//for progress notification
	double initialCMD = 0, classMovingDist = 0;

	//let's start
	bool meansHaveMoved = false;
	int iteration = 0;
	do
	{
		meansHaveMoved = false;
		++iteration;
		{
			for (unsigned i=0; i<n; ++i)
			{
				uchar minK = 0;

				ScalarType V = theCloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(V))
				{
					minDistsToMean[i] = fabs(theKMeans[minK]-V);

					//on recherche le centre de cluster le plus proche
					for (uchar j=1; j<K; ++j)
					{
						ScalarType distToMean = fabs(theKMeans[j]-V);
						if (distToMean<minDistsToMean[i])
						{
							minDistsToMean[i]=distToMean;
							minK = j;
						}
					}
				}

				belongings[i] = minK;
				minDistsToMean[i] = V;
			}
		}

		//on peut maintenant recalculer les centres des clusters
		//Console::print("[Kmeans] Calcul des centres ...\n");
		//Fl::wait(1.0);

		theOldKNums = theKNums;
		std::fill(theKSums.begin(),theKSums.end(),static_cast<ScalarType>(0));
		std::fill(theKNums.begin(),theKNums.end(),static_cast<unsigned>(0));
		{
			for (unsigned i=0; i<n; ++i)
			{
				if (minDistsToMean[i] >= 0.0) //must be a valid value!
				{
					theKSums[belongings[i]] += minDistsToMean[i];
					++theKNums[belongings[i]];
				}
			}
		}

		classMovingDist = 0.0;
		{
			for (uchar j=0; j<K; ++j)
			{
				ScalarType newMean = (theKNums[j]>0 ? theKSums[j]/(ScalarType)theKNums[j] : theKMeans[j]);

				if (theOldKNums[j] != theKNums[j])
					meansHaveMoved = true;

				classMovingDist += static_cast<double>(fabs(theKMeans[j] - newMean));

				theKMeans[j] = newMean;
			}
		}

		if (progressCb)
		{
			if (iteration == 1)
			{
				progressCb->reset();
				progressCb->setMethodTitle("KMeans");
				char buffer[256];
				sprintf(buffer,"K=%i",K);
				progressCb->setInfo(buffer);
				progressCb->start();
				initialCMD = classMovingDist;
			}
			else
			{
				progressCb->update(static_cast<float>((1.0 - classMovingDist/initialCMD) * 100.0));
			}
		}
	}
	while (meansHaveMoved);

	//on met a jour les distances pour refleter la segmentation
	std::vector<ScalarType> mins,maxs;
	try
	{
		mins.resize(K,maxV);
		maxs.resize(K,minV);
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	//on recherche les mins et maxs de chaque cluster
	{
		for (unsigned i=0;i<n;++i)
		{
			ScalarType V = theCloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
				if (V < mins[belongings[i]])
					mins[belongings[i]] = V;
				else if (V > maxs[belongings[i]])
					maxs[belongings[i]] = V;
			}
		}
	}

	//derniere verif
	{
		for (uchar j=0; j<K; ++j)
			if (theKNums[j] == 0)
				mins[j] = maxs[j] = -1.0;
	}

	//format de sortie
	{
		for (uchar j=0; j<K; ++j)
		{
			kmcc[j].mean = theKMeans[j];
			kmcc[j].minValue = mins[j];
			kmcc[j].maxValue = maxs[j];
		}
	}

	if (progressCb)
        progressCb->stop();

	return true;
}

ScalarType ScalarFieldTools::computeMeanScalarValue(GenericCloud* theCloud)
{
	double meanValue = 0.0;
	unsigned count=0;

	for (unsigned i=0; i<theCloud->size(); ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			meanValue += (double)V;
			++count;
		}
	}

	return (count ? (ScalarType)(meanValue/(double)count) : 0);
}

ScalarType ScalarFieldTools::computeMeanSquareScalarValue(GenericCloud* theCloud)
{
	double meanValue = 0.0;
	unsigned count=0;

	for (unsigned i=0; i<theCloud->size(); ++i)
	{
		ScalarType V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			meanValue += (double)V*(double)V;
			++count;
		}
	}

	return (count ? (ScalarType)(meanValue/(double)count) : 0);
}
