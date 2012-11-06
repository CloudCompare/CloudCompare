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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "ScalarFieldTools.h"

#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "ReferenceCloud.h"
#include "GenericProgressCallback.h"
#include "GenericChunkedArray.h"
#include "ScalarField.h"

#include <assert.h>

using namespace CCLib;

void ScalarFieldTools::razDistsToHiddenValue(const CCVector3& aPoint, DistanceType& aScalarValue)
{
	aScalarValue=HIDDEN_VALUE;
}

void ScalarFieldTools::razDistsToBigValue(const CCVector3& aPoint, DistanceType& aScalarValue)
{
	aScalarValue=BIG_VALUE;
}

//mise à zéro des distances
void ScalarFieldTools::razDistsToZero(const CCVector3& aPoint, DistanceType& aScalarValue)
{
	aScalarValue=0.0;
}

int ScalarFieldTools::computeScalarFieldGradient(GenericIndexedCloudPersist* theCloud, bool signedSF, bool euclidianDistances, bool sameInAndOutScalarField, GenericProgressCallback* progressCb, DgmOctree* theCloudOctree)
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

	ScalarField* theGradientNorms = new ScalarField("gradient norms",true);
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
	//mode champs scalaires "IN" et "OUT" dfférents (par defaut)
	{
		//on initialise les distances (IN - en écriture) pour recevoir les normes du gradient
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
	void* additionalParameters[4];
	additionalParameters[0] = (void*)&euclidianDistances;
	additionalParameters[1] = (void*)&radius;
	additionalParameters[2] = (void*)_theGradientNorms;
	additionalParameters[3] = (void*)&signedSF;

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
	bool signedSF                                           = *((bool*)additionalParameters[3]);

	unsigned i,j,n;

	//nombre de points dans la cellule courante
	n = cell.points->size();

	//structures pour la recherche de voisinages SPECIFIQUES
	DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.truncatedCellCode = cell.truncatedCode;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//on connait déjà les points de la première cellule
	//(c'est la cellule qu'on est en train de traiter !)
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}
	DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	for (j=0;j<n;++j,++it)
	{
		it->point = cell.points->getPointPersistentPtr(j);
		it->pointIndex = cell.points->getPointGlobalIndex(j);
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	unsigned counter;
	CCVector3 u,v;
	PointCoordinateType norm2;
	DistanceType d1,d2,gN;
	const GenericIndexedCloudPersist* cloud = cell.points->getAssociatedCloud();

	for (i=0;i<n;++i)
	{
		gN = HIDDEN_VALUE;

		d1 = cell.points->getPointScalarValue(i);
        if (d1>=0.0 || (signedSF && d1<BIG_VALUE))
		{
			 cell.points->getPoint(i,nNSS.queryPoint);

			//on extrait un voisinage autour du point
			int k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,true);

            //if more than one neighbour (the query point itself)
			if (k>1)
			{
				v.x=v.y=v.z=0.0;
				counter=0;

				//j=1 because the first point is the query point itself --> contribution = 0
				for (j=1;j<(unsigned)k;++j)
				{
					d2 = cloud->getPointScalarValue(nNSS.pointsInNeighbourhood[j].pointIndex);
					if (d2>=0.0 || (signedSF && d2<BIG_VALUE))
					{
						u = *nNSS.pointsInNeighbourhood[j].point - nNSS.queryPoint;
						norm2 = u.norm2();

						if (norm2>ZERO_TOLERANCE)
						{
                            d2 -= d1;
							if (!euclidianDistances || d2*d2 < 1.01*norm2) //warning: here 'd2'=d2-d1
							{
								v += u * (d2/norm2); //warning: here 'd2'=d2-d1
								++counter;
							}
						}
					}
				}

				if (counter>0)
					gN = v.norm()/(PointCoordinateType)counter;
			}
		}

		if (theGradientNorms)
			//mode champ scalaire "IN" et "OUT" identique
			theGradientNorms->setValue(cell.points->getPointGlobalIndex(i),gN);
		else
			//mode champs scalaires "IN" et "OUT" différents
			cell.points->setPointScalarValue(i,gN);
	}

	return true;
}

bool ScalarFieldTools::applyScalarFieldGaussianFilter(float sigma,
													  GenericIndexedCloudPersist* theCloud,
													  bool signedSF,
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

    void* additionalParameters[3];
	additionalParameters[0] = (void*)&sigma;
	additionalParameters[1] = (void*)&signedSF;
    additionalParameters[2] = (void*)&sigmaSF;

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
// [1] -> (bool*) signedSF : specify whether the SF is signed or not
// [2] -> (float*) sigmaSF : used when in "bilateral modality" - if -1 pure gaussian filtering is performed
bool ScalarFieldTools::computeCellGaussianFilter(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	float sigma     = *((float*)additionalParameters[0]);
	bool signedSF   = *((bool*)additionalParameters[1]);
    float sigmaSF	= *((float*)additionalParameters[2]);

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
                DistanceType val = cloud->getPointScalarValue(it->pointIndex);
                //scalar value must be valid
                if (val>=0.0 || (signedSF && val < BIG_VALUE))
                {
                    meanValue += (double)val * weight;
                    wSum += weight;
                }
            }

			DistanceType newValue;
            if (wSum>0.0)
                newValue = (DistanceType)(meanValue / wSum);
            else
                newValue = (DistanceType)(signedSF ? BIG_VALUE : HIDDEN_VALUE);

            cell.points->setPointScalarValue(i,newValue);
        }
    }

    //Bilateral Filtering using the second sigma parameters on values (when given)
    else
    {
        for (unsigned i=0;i<n;++i) //for each point in cell
        {
            DistanceType queryValue = cell.points->getPointScalarValue(i); //scalar of the query point

            //we get the points inside a spherical neighbourhood (radius: '3*sigma')
            cell.points->getPoint(i,nNSS.queryPoint);
            unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);

            //each point adds a contribution weighted by its distance to the sphere center
            it = nNSS.pointsInNeighbourhood.begin();
            double meanValue = 0.0;
            double wSum = 0.0;
            for (unsigned j=0;j<k;++j,++it)
            {
                DistanceType val = cloud->getPointScalarValue(it->pointIndex);
				DistanceType dSF = queryValue - val;
                double weight = exp(-(it->squareDist)/sigma2) * exp(-(dSF*dSF)/sigmaSF2);
                //scalar value must be valid
                if (val>=0.0 || (signedSF && val < BIG_VALUE))
                {
                    meanValue += (double)val * weight;
                    wSum += weight;
                }
            }

			DistanceType newValue;
            if (wSum>0.0)
                newValue = (DistanceType)(meanValue / wSum);
            else
                newValue = (DistanceType)(signedSF ? BIG_VALUE : HIDDEN_VALUE);

            cell.points->setPointScalarValue(i,newValue);
        }

    }

	return true;
}

void ScalarFieldTools::multiplyScalarFields(GenericIndexedCloud* firstCloud, GenericIndexedCloud* secondCloud, GenericProgressCallback* progressCb)
{
	if ((!firstCloud)||(!secondCloud)) return;

	unsigned n1 = firstCloud->size();
	if ((n1 != secondCloud->size())||(n1==0))
	{
		//printf("[ScalarFieldTools::multiplyScalarFields] Clouds must have the same size !\n");
		return;
	}

	DistanceType V1,V2;
	unsigned i=0;
	for (;i<n1;++i)
	{
		V1 = firstCloud->getPointScalarValue(i);
		V2 = secondCloud->getPointScalarValue(i);
		firstCloud->setPointScalarValue(i,V1*V2);
	}
}

void ScalarFieldTools::computeScalarFieldExtremas(const GenericCloud* theCloud, DistanceType& minV, DistanceType& maxV, bool includeNegValues)
{
    assert(theCloud);

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
        return;

	//recherce des extremas
	DistanceType V;
	minV = maxV = 0.0;

	bool firstValue=true;

	for (unsigned i=0;i<numberOfPoints;++i)
	{
		V = theCloud->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
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

unsigned ScalarFieldTools::countScalarFieldPositiveValues(const GenericCloud* theCloud)
{
    assert(theCloud);

	DistanceType V;
	unsigned count = 0;
	unsigned i,n = theCloud->size();

	for (i=0;i<n;++i)
	{
		V = theCloud->getPointScalarValue(i);
		if (V>=0.0)
		 ++count;
	}

	return count;
}

void ScalarFieldTools::computeScalarFieldHistogram(const GenericCloud* theCloud, unsigned numberOfClasses, std::vector<int>& histo, bool includeNegValues)
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
	DistanceType minV,maxV;
	computeScalarFieldExtremas(theCloud,minV,maxV,includeNegValues);

	//on en déduit le pas de l'historgramme
	DistanceType invStep = (maxV>minV ? (DistanceType)numberOfClasses / (maxV-minV) : 0.0f);

	//projection des valeurs dans l'histo
	for (unsigned i=0;i<theCloud->size();++i)
	{
		DistanceType V = theCloud->getPointScalarValue(i);

		if (includeNegValues || V >= 0.0)
		{
            int aimClass = (int)((V-minV)*invStep);
            if (aimClass == (int)numberOfClasses)
                --aimClass; //attention à la frontière sup.

            ++histo[aimClass];
		}
	}
}

bool ScalarFieldTools::computeKmeans(const GenericCloud* theCloud, uchar K, KMeanClass kmcc[], bool includeNegValues, GenericProgressCallback* progressCb)
{
	assert(theCloud);

	unsigned n = theCloud->size();
	if (n==0)
        return false;

	//on a besoin de mémoire ici !
	DistanceType* theKMeans = new DistanceType[n]; //le centre des K clusters
	uchar* belongings = new uchar[n]; //l'appartenance d'un point à un cluster
	uchar* _belongings = 0;
	DistanceType* minDistsToMean = new DistanceType[n];  //les distances au centre de cluster le plus proche

	DistanceType* theKSums = new DistanceType[K]; //le cumuls de distance des k clusters
	unsigned* theKNums = new unsigned[K]; //le nombre de point par cluster
	unsigned* theOldKNums = new unsigned[K]; //le nombre de point par cluster (ancien)

	//on récupère les extremas
	DistanceType V,minV,maxV;
	computeScalarFieldExtremas(theCloud, minV, maxV,includeNegValues);

	//initialisation des K-means
	DistanceType delta = maxV - minV;
	DistanceType step = delta / DistanceType(K);

	unsigned i;
	uchar j;
	for (j=0;j<K;++j)
        theKMeans[j] = minV + DistanceType(j)*step;

	//on lance l'iterration
	bool meansHaveMoved = true;
	int iteration = 0;

	float initialCMD=0.0,classMovingDist=0.0;

	//printf("[Kmeans] Calcul des K-Means avec K=%i\n",K);
	while (meansHaveMoved)
	{
		meansHaveMoved = false;
		++iteration;

		//printf("[Kmeans] Iteration %i\n",iteration);

		_belongings = belongings;

		uchar minK;
		DistanceType distToMean,newMean;
		DistanceType *_minDistToMean = minDistsToMean;

		//pour chaque point
		//Console::print("[Kmeans] Calcul des distances ...\n");
		//Fl::wait(1.0);
		for (i=0;i<n;++i)
		{
			minK = 0;

			V = theCloud->getPointScalarValue(i);
			if (includeNegValues || V>= 0.0)
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

		memset(theKSums,0,sizeof(DistanceType)*K);
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
			newMean = (theKNums[j]>0 ? theKSums[j]/(DistanceType)theKNums[j] : theKMeans[j]);

			//printf("[Kmeans] mean #%i = %f (%i pts, delta=%f)\n",j,newMean,theKNums[j],newMean-theKMeans[j]);
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

	//on met à jour les distances pour refléter la segmentation
	DistanceType* mins = new DistanceType[K];
	DistanceType* maxs = new DistanceType[K];

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
		if (includeNegValues || V>=0.0)
		{
            if (V<mins[*_belongings])
                mins[*_belongings] = V;
            else if (V>maxs[*_belongings])
                maxs[*_belongings] = V;
		}

		++_belongings;
	}

	//dernière verif
	for (j=0;j<K;++j)
        if (theKNums[j]==0)
            mins[j]=maxs[j]=-1.0;

	//format de sortie
	for (j=0;j<K;++j)
	{
		kmcc[j].mean = theKMeans[j];
		kmcc[j].minValue = mins[j];
		kmcc[j].maxValue = maxs[j];

		//printf("[KMeans] cluster #%i : [%f,%f] (%i points)\n",j,mins[j],maxs[j],theKNums[j]);
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
