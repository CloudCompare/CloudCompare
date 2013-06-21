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

using namespace CCLib;

void ScalarFieldTools::SetScalarValueToNaN(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = NAN_VALUE;
}

void ScalarFieldTools::SetScalarValueToZero(const CCVector3& P, ScalarType& scalarValue)
{
	scalarValue = 0;
}

int ScalarFieldTools::computeScalarFieldGradient(GenericIndexedCloudPersist* theCloud, bool euclidianDistances, bool sameInAndOutScalarField, GenericProgressCallback* progressCb, DgmOctree* theCloudOctree)
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
	float radius = theOctree->getCellSize(octreeLevel);
	void* additionalParameters[3] = {	(void*)&euclidianDistances,
										additionalParameters[1] = (void*)&radius,
										additionalParameters[2] = (void*)_theGradientNorms
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

bool ScalarFieldTools::computeMeanGradientOnPatch(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	bool euclidianDistances									= *((bool*)additionalParameters[0]);
	float radius											= *((float*)additionalParameters[1]);
	ScalarField* theGradientNorms							= (ScalarField*)additionalParameters[2];

	//nombre de points dans la cellule courante
	unsigned n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.truncatedCellCode = cell.truncatedCode;
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
		for (unsigned j=0;j<n;++j,++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

	for (unsigned i=0;i<n;++i)
	{
		ScalarType gN = NAN_VALUE;

		ScalarType d1 = cell.points->getPointScalarValue(i);

        if (ScalarField::ValidValue(d1))
		{
			 cell.points->getPoint(i,nNSS.queryPoint);

			//on extrait un voisinage autour du point
			int k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,true);

            //if more than one neighbour (the query point itself)
			if (k>1)
			{
				double sum[3]={0.0,0.0,0.0};
				unsigned counter=0;

				//j=1 because the first point is the query point itself --> contribution = 0
				for (int j=1; j<k; ++j)
				{
					ScalarType d2 = cloud->getPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex);
					if (ScalarField::ValidValue(d2))
					{
						CCVector3 u = *nNSS.pointsInNeighbourhood[j].point - nNSS.queryPoint;
						PointCoordinateType norm2 = u.norm2();

						if (norm2 > ZERO_TOLERANCE)
						{
                            ScalarType dd = d2 - d1;
							if (!euclidianDistances || dd*dd < 1.01 * norm2)
							{
								dd /= norm2;
								sum[0] += (double)(u.x * dd); //warning: and here 'dd'=dd/norm2 ;)
								sum[1] += (double)(u.y * dd);
								sum[2] += (double)(u.z * dd);
								++counter;
							}
						}
					}
				}

				if (counter != 0)
					gN = (ScalarType)(sqrt(sum[0]*sum[0]*+sum[1]*sum[1]+sum[2]*sum[2])/(double)counter);
			}
		}

		if (theGradientNorms)
			//mode champ scalaire "IN" et "OUT" identique
			theGradientNorms->setValue(cell.points->getPointGlobalIndex(i),gN);
		else
			//mode champs scalaires "IN" et "OUT" differents
			cell.points->setPointScalarValue(i,gN);
	}

	return true;
}

bool ScalarFieldTools::applyScalarFieldGaussianFilter(float sigma,
													  GenericIndexedCloudPersist* theCloud,
													  float sigmaSF,
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
// [0] -> (float*) sigma : gauss function sigma
// [1] -> (float*) sigmaSF : used when in "bilateral modality" - if -1 pure gaussian filtering is performed
bool ScalarFieldTools::computeCellGaussianFilter(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	float sigma     = *((float*)additionalParameters[0]);
    float sigmaSF	= *((float*)additionalParameters[1]);

    //we use only the squared value of sigma
	float sigma2 = 2.0f*sigma*sigma;
	float radius = 3.0f*sigma; //2.5 sigma > 99%

	//we use only the squared value of sigmaSF
    float sigmaSF2 = 2.0f*sigmaSF*sigmaSF;

	//number of points inside the current cell
	unsigned n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.truncatedCellCode = cell.truncatedCode;
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
		for (unsigned i=0;i<n;++i,++it)
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
        for (unsigned i=0;i<n;++i) //for each point in cell
        {
            //we get the points inside a spherical neighbourhood (radius: '3*sigma')
            cell.points->getPoint(i,nNSS.queryPoint);
            unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

            //each point adds a contribution weighted by its distance to the sphere center
            it = nNSS.pointsInNeighbourhood.begin();
            double meanValue = 0.0;
            double wSum = 0.0;
            for (unsigned j=0;j<k;++j,++it)
            {
                double weight = exp(-(it->squareDist)/sigma2); //PDF: -exp(-(x-mu)^2/(2*sigma^2))
                ScalarType val = cloud->getPointScalarValue(it->pointIndex);
                //scalar value must be valid
				if (ScalarField::ValidValue(val))
                {
                    meanValue += (double)val * weight;
                    wSum += weight;
                }
            }

			ScalarType newValue = (wSum > 0.0 ? (ScalarType)(meanValue / wSum) : NAN_VALUE);

            cell.points->setPointScalarValue(i,newValue);
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
            unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

            //each point adds a contribution weighted by its distance to the sphere center
            it = nNSS.pointsInNeighbourhood.begin();
            double meanValue = 0.0;
            double wSum = 0.0;
            for (unsigned j=0;j<k;++j,++it)
            {
                ScalarType val = cloud->getPointScalarValue(it->pointIndex);
				ScalarType dSF = queryValue - val;
                double weight = exp(-(it->squareDist)/sigma2) * exp(-(dSF*dSF)/sigmaSF2);
                //scalar value must be valid
				if (ScalarField::ValidValue(val))
                {
                    meanValue += (double)val * weight;
                    wSum += weight;
                }
            }

            cell.points->setPointScalarValue(i,wSum > 0.0 ? (ScalarType)(meanValue / wSum) : NAN_VALUE);
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

bool ScalarFieldTools::computeKmeans(const GenericCloud* theCloud, uchar K, KMeanClass kmcc[], GenericProgressCallback* progressCb)
{
	assert(theCloud);

	unsigned n = theCloud->size();
	if (n==0)
        return false;

	//on a besoin de memoire ici !
	ScalarType* theKMeans = new ScalarType[n]; //le centre des K clusters
	uchar* belongings = new uchar[n]; //l'appartenance d'un point a un cluster
	uchar* _belongings = 0;
	ScalarType* minDistsToMean = new ScalarType[n];  //les distances au centre de cluster le plus proche

	ScalarType* theKSums = new ScalarType[K]; //le cumuls de distance des k clusters
	unsigned* theKNums = new unsigned[K]; //le nombre de point par cluster
	unsigned* theOldKNums = new unsigned[K]; //le nombre de point par cluster (ancien)

	//on recupere les extremas
	ScalarType V,minV,maxV;
	computeScalarFieldExtremas(theCloud, minV, maxV);

	//initialisation des K-means
	ScalarType delta = maxV - minV;
	ScalarType step = delta / ScalarType(K);

	unsigned i;
	uchar j;
	for (j=0;j<K;++j)
        theKMeans[j] = minV + ScalarType(j)*step;

	//on lance l'iterration
	bool meansHaveMoved = true;
	int iteration = 0;

	float initialCMD=0.0,classMovingDist=0.0;

	while (meansHaveMoved)
	{
		meansHaveMoved = false;
		++iteration;

		_belongings = belongings;

		uchar minK;
		ScalarType distToMean,newMean;
		ScalarType *_minDistToMean = minDistsToMean;

		//pour chaque point
		//Console::print("[Kmeans] Calcul des distances ...\n");
		//Fl::wait(1.0);
		for (i=0;i<n;++i)
		{
			minK = 0;

			V = theCloud->getPointScalarValue(i);
			if (ScalarField::ValidValue(V))
			{
                *_minDistToMean = fabs(theKMeans[minK]-V);

                //on recherche le centre de cluster le plus proche
                for (j=1;j<K;++j)
                {
                    distToMean = fabs(theKMeans[j]-V);
                    if (distToMean<*_minDistToMean)
                    {
                        *_minDistToMean=distToMean;
                        minK = j;
                    }
                }
			}

			*_belongings = minK;
			++_belongings;
			*_minDistToMean = V;
			++_minDistToMean;
		}

		//on peut maintenant recalculer les centres des clusters
		//Console::print("[Kmeans] Calcul des centres ...\n");
		//Fl::wait(1.0);

		_minDistToMean = minDistsToMean;
		_belongings = belongings;

		memset(theKSums,0,sizeof(ScalarType)*K);
		memcpy(theOldKNums,theKNums,sizeof(unsigned)*K);
		memset(theKNums,0,sizeof(unsigned)*K);

		for (i=0;i<n;++i)
		{
		    if (*_minDistToMean >= 0.0) //must be a valid value!
		    {
                theKSums[*_belongings] += *_minDistToMean;
                ++theKNums[*_belongings];
		    }

			++_belongings;
			++_minDistToMean;
		}

		classMovingDist = 0.0;
		for (j=0;j<K;++j)
		{
			newMean = (theKNums[j]>0 ? theKSums[j]/(ScalarType)theKNums[j] : theKMeans[j]);

			if (theOldKNums[j] != theKNums[j])
                meansHaveMoved=true;

			classMovingDist += fabs(theKMeans[j] - newMean);

			theKMeans[j] = newMean;
		}

		if (progressCb)
		{
			if (iteration==1)
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
				progressCb->update((1.0f-classMovingDist/initialCMD)*100.0f);
			}
		}
	}

	//on met a jour les distances pour refleter la segmentation
	ScalarType* mins = new ScalarType[K];
	ScalarType* maxs = new ScalarType[K];

	for (j=0;j<K;++j)
	{
		mins[j]=maxV;
		maxs[j]=minV;
	}

	//on recherche les mins et maxs de chaque cluster
	_belongings = belongings;
	for (i=0;i<n;++i)
	{
		V = theCloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
            if (V<mins[*_belongings])
                mins[*_belongings] = V;
            else if (V>maxs[*_belongings])
                maxs[*_belongings] = V;
		}

		++_belongings;
	}

	//derniere verif
	for (j=0;j<K;++j)
        if (theKNums[j]==0)
            mins[j]=maxs[j]=-1.0;

	//format de sortie
	for (j=0;j<K;++j)
	{
		kmcc[j].mean = theKMeans[j];
		kmcc[j].minValue = mins[j];
		kmcc[j].maxValue = maxs[j];
	}

	if (progressCb)
        progressCb->stop();

	delete[] mins;
	delete[] maxs;
	delete[] theKMeans;
	delete[] theKSums;
	delete[] belongings;
	delete[] minDistsToMean;
	delete[] theKNums;
	delete[] theOldKNums;

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
