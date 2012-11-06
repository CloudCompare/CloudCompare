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

#include "StatisticalTestingTools.h"

#include "ReferenceCloud.h"
#include "DgmOctreeReferenceCloud.h"
#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "GenericDistribution.h"
#include "DgmOctree.h"
#include "GenericProgressCallback.h"

// Chi2 computation stuff
#include "Chi2Helper.h"

#include <assert.h>

using namespace CCLib;

//! Max computable Chi2 distance
#define CHI2_MAX 1e7

//! Min computable Chi2 distance
#define CHI2_MIN 1e-6

//calcul de la "distance" du Chi2 entre un échantillon Yk et une distribution "distrib"
//On peut fournir un tableau (histogramme) déja alloué (dumpHisto) pour aller plus vite
//Le nombe minimal de classes est 2 (pour éviter que le résultat du test soit nul, et de toutes façons, un test du Chi2 à 0 ddl n'existe pas ;)
double StatisticalTestingTools::computeChi2Dist(const GenericDistribution* distrib, const GenericCloud* Yk, unsigned numberOfClasses, bool includeNegValues, unsigned* dumpHisto, double* npis)
{
    assert(distrib && Yk);
	if (!distrib->isValid())
		return -1.0;

	unsigned n = Yk->size();

	if (n==0 || numberOfClasses<2)
		return -1.0;

	//on va calculer la fonction de repartition
	DistanceType V, minV=0.0, maxV=0.0;
	unsigned i,numberOfElements=0;
	bool firstValue=true;
	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V >= 0.0)
		{
		    if (firstValue)
		    {
		        minV=maxV=V;
		        firstValue=false;
		    }
		    else
		    {
                if (V>maxV)
                    maxV=V;
                else if (V<minV)
                    minV=V;
		    }
		    ++numberOfElements;
		}
	}

	if (firstValue)
        return -1.0;

    if (!includeNegValues)
        minV=0.0; //for only positive values, it's better if the histogram starts at 0!

	//on s'assure que l'intervale de l'histogramme englobe bien toute la distribution théorique
    if (maxV>minV)
    {
        while (distrib->computeP(maxV) > ZERO_TOLERANCE)
            maxV += 0.05f*(maxV-minV);

        if (includeNegValues)
            while (distrib->computeP(minV) > ZERO_TOLERANCE)
                minV -= 0.05f*(maxV-minV);
    }

	DistanceType dV = maxV-minV;
	DistanceType step = dV/(DistanceType)numberOfClasses;
	if (step < ZERO_TOLERANCE)
        return -1.0;
	DistanceType coef = 1.0f/step;

	unsigned* count = (dumpHisto ? dumpHisto : new unsigned[numberOfClasses]);
	memset(count,0,sizeof(unsigned)*numberOfClasses);

	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V >= 0.0)
		{
            int aim = int(floor((V-minV)*coef));
            //pour éviter les problèmes de bords
            if (aim>=(int)numberOfClasses)
                aim=numberOfClasses-1;
            //accumulation
            ++count[aim];
		}
	}

	//on calcule enfin la "distance au carré" du Chi2
	double npi,temp,D2 = 0.0;
	DistanceType x1,x2=minV;
	double p1,p2=distrib->computePfromZero(x2);

	for (unsigned k=0;k<numberOfClasses;++k)
	{
		x1 = x2;
		x2 = x1+step;
		p1 = p2;
		p2 = distrib->computePfromZero(x2);
		npi = (p2-p1)*(double)numberOfElements;
		if (npis)
            npis[k]=npi;

		if (count[k]==0)
		{
			D2 += npi;
		}
		//pour éviter les overflows !
		else if (npi < CHI2_MIN)
		{
			D2 = CHI2_MAX;
			break;
		}
		else
		{
			temp = (double)((DistanceType)count[k] - npi);
			D2 += temp*(temp/(double)npi);

			if (D2 > CHI2_MAX)
			{
				D2 = CHI2_MAX;
				break;
			}
		}
	}

	if (!dumpHisto)
        delete[] count;

	return D2;
}

//Version plus correcte au sens de la théorie du Test du Chi2, mais aussi beaucoup plus lente !
//Elle cherche à imposer que chaque classe ait un n.pi >= 5 (n = nombre total d'éléments, pi = la probabilité de la classe)
//Il faut par contre gérer le changement local du nombre final de classes (qui change donc le seuil du Chi2)
double StatisticalTestingTools::computeAdaptativeChi2Dist(const GenericDistribution* distrib, const GenericCloud* Yk, unsigned numberOfClasses, unsigned &finalNumberOfClasses, bool includeNegValues, bool forceZeroAsMin, unsigned* dumpHisto, double* npis)
{
    assert(distrib && Yk);
	if (!distrib->isValid())
		return -1.0;

	unsigned n = Yk->size();
	if (n==0)
        return -1.0;

	//on va calculer la fonction de repartition
	DistanceType V,minV=0.0,maxV=0.0;
	unsigned i,numberOfElements=0;

	//on cherche les valeurs min et max
	bool firstValue=true;
	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V >= 0.0)
		{
		    if (firstValue)
		    {
		        minV=maxV=V;
		        firstValue=false;
		    }
		    else
		    {
                if (V>maxV)
                    maxV=V;
                else if (V<minV)
                    minV=V;
		    }
		    ++numberOfElements;
		}
	}

	if (firstValue)
        return -1.0;

    if (!includeNegValues && forceZeroAsMin)
        minV=0.0; //for only positive values, it's better if the histogram starts at 0!

	//on s'assure que l'intrevale de l'histogramme englobe bien toute la distribution théorique
	//while ((distrib->computeP(maxV)>ZERO_TOLERANCE)) maxV *= 1.5;

	//si on doit determiner automatiquement le nombre de classes
	if (numberOfClasses==0)
        numberOfClasses=(unsigned)ceil(sqrt((float)numberOfElements));
	if (numberOfClasses<2)
        return -2.0; //pas assez de points/classes

	DistanceType dV = maxV-minV;
    DistanceType step = dV/(DistanceType)numberOfClasses;
	if (step < ZERO_TOLERANCE)
        return -1.0;
	DistanceType coef = 1.0f/step;

	//le tableau de stockage de l'histogramme peut avoir été passé en argument de la fonction
	unsigned* count = (dumpHisto ? dumpHisto : new unsigned[numberOfClasses]);
	memset(count,0,sizeof(unsigned)*numberOfClasses);

	//on calcule l'histogramme de "numberOfClasses" classes entre minV et maxV
	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
		{
			int aim = int(floor((V-minV)*coef)); //la colonne de l'histogramme à accumuler
			//pour éviter les problèmes de bords
			if (aim==(int)numberOfClasses)
                --aim;
			//accumulation
			count[aim]++;
		}
	}

	//structure de liste chainée qui permettra la compression des clases si besoin
	Chi2Element* root = new Chi2Element;
	root->pred = 0;
	root->next = 0;
	Chi2Element *currentCE = root;

	//on calcule enfin la "distance au carré" du Chi2
	DistanceType x1,x2=minV;
	double p1,p2=distrib->computePfromZero(x2);

	for (unsigned k=0;k<numberOfClasses;++k)
	{
		x1 = x2;
		x2 = x1+step;
		p1 = p2;
		p2 = distrib->computePfromZero(x2);

		//on créé le nouvel élement de la liste chainée
		currentCE->next = new Chi2Element;
		currentCE->next->pred = currentCE;
		//on avance d'un cran
		currentCE = currentCE->next;
		//on initialise les valeurs du nouvel élément
		currentCE->next = 0;
		currentCE->n = count[k];
		//currentCE->pi = max(p2-p1,0.0);
		currentCE->pi = p2-p1;
		if (npis)
            npis[k]= currentCE->pi * (double)numberOfElements;
	}

	//COMPRESSION DES CLASSES DE L'HISTOGRAMME
	//on intialise "root->pi" avec la valeur minimale acceptable "K/n" (K=5 généralement, mais pourrait être 3 ou 1 en queue !)
	root->pi = 5.0/(double)numberOfElements;
	Chi2Element* minCE;

	finalNumberOfClasses = numberOfClasses;
	while (finalNumberOfClasses>2)
	{
		//on cherche la plus petite valeur de "npi"
		currentCE = minCE = root;
		while (currentCE->next)
		{
			currentCE = currentCE->next;
			if (currentCE->pi < minCE->pi)
				minCE = currentCE;
		}

		//on n'a pas bougé, donc il n'y a plus/pas d'amas trop petit
		if (minCE == root)
			break;

		//sinon, il faut fusionner la valeur la plus petite avec son voisin (le plus petit pour équilibrer un peu les amas)

		//cas où "pred" existe (du moins est "valabe", i.e. différent de "root")
		if (minCE->pred != root)
		{
			//et "next" existe
			if (minCE->next)
			{
				//il faut fusionner avec "pred"
				if (minCE->pred->pi < minCE->next->pi)
				{
					minCE->pred->pi += minCE->pi;
					minCE->pred->n += minCE->n;
					minCE->pred->next = minCE->next;
					minCE->next->pred = minCE->pred;
				}
				else //il faut fusionner avec "next"
				{
					minCE->next->pi += minCE->pi;
					minCE->next->n += minCE->n;
					minCE->next->pred = minCE->pred;
					minCE->pred->next = minCE->next;
				}
			}
			//il faut fusionner avec "pred" (sans next)
			else
			{
				minCE->pred->pi += minCE->pi;
				minCE->pred->n += minCE->n;
				minCE->pred->next = 0;
			}
		}
		else
		//il faut fusionner avec "next" (pred == root)
		{
			minCE->next->pi += minCE->pi;
			minCE->next->n += minCE->n;
			minCE->next->pred = minCE->pred/*root*/;
			/*root*/minCE->pred->next = minCE->next;
		}

		delete minCE;
		finalNumberOfClasses--;
	}

	//calcul du Chi2 avec les nouvelles classes
	currentCE = root;
	double npi,temp,D2=0.0;
	while (currentCE->next)
	{
		currentCE = currentCE->next;
		delete currentCE->pred;
		currentCE->pred=0;

		if (D2 < CHI2_MAX)
		{
			npi = currentCE->pi * (double)numberOfElements;
			if (npi < CHI2_MIN)
			{
				D2 = CHI2_MAX;
			}
			else
			{
				temp = (double)currentCE->n - npi;
				D2 += temp*(temp/npi);
			}
		}
	}
	delete currentCE;

	if (!dumpHisto)
        delete[] count;

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
                                                              bool includeNegValues /*= false*/,
                                                              GenericProgressCallback* progressCb,
                                                              DgmOctree* _theOctree)
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

	unsigned numberOfChi2Classes = (unsigned)sqrt((double)numberOfNeighbours);

	//l'histogramme pour le calcul du Chi2
	unsigned* dumpHisto = new unsigned[numberOfChi2Classes];
	if (!dumpHisto)
	{
		if (!_theOctree)
			delete theOctree;
		return -3.0;
	}

	//additionnal parameters for local process
	void* additionalParameters[5];
	additionalParameters[0] = (void*)distrib;
	additionalParameters[1] = (void*)&numberOfNeighbours;
	additionalParameters[2]	= (void*)&numberOfChi2Classes;
	additionalParameters[3]	= (void*)dumpHisto;
	additionalParameters[4]	= (void*)&includeNegValues;

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
		//no user cancellation?
		if (!progressCb || !progressCb->isCancelRequested())
		{
			//theoretical Chi2 fractile
			maxChi2 = computeChi2Fractile(pTrust, numberOfChi2Classes-1);
			//printf("MaxChi2 (%i ddl / p=%1.5f) = %f\n",numberOfChi2Classes-1,pTrust,maxChi2);
			maxChi2 = sqrt(maxChi2); //on travaille avec les racines carrées des distances du Chi2
		}
	}

	delete[] dumpHisto;
	dumpHisto=0;

	if (!_theOctree)
        delete theOctree;

	return maxChi2;
}

//FONCTION "CELLULAIRE" DE CALCUL DE LA DISTANCE DU CHI2
bool StatisticalTestingTools::computeLocalChi2DistAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	GenericDistribution* statModel		= (GenericDistribution*)additionalParameters[0];
	unsigned numberOfNeighbours         = *(unsigned*)additionalParameters[1];
	unsigned numberOfChi2Classes		= *(unsigned*)additionalParameters[2];
	unsigned* dumpHisto					= (unsigned*)additionalParameters[3];
	bool includeNegValues               = *(bool*)additionalParameters[4];

	//nombre de points dans la cellule courante
	unsigned i,j,n = cell.points->size();

	DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level												= cell.level;
	nNSS.minNumberOfNeighbors								= numberOfNeighbours;
	nNSS.truncatedCellCode									= cell.truncatedCode;
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

	//VERSION "STANDARD"
	for (i=0;i<n;++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);
		DistanceType D = cell.points->getPointScalarValue(i);

		if (D>=0.0)
		{
			//nNSS.theNearestPoints.clear();

			unsigned k = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS,false,true);
			if (k>numberOfNeighbours)
				k=numberOfNeighbours;

			DgmOctreeReferenceCloud neighboursCloud(&nNSS.pointsInNeighbourhood,k);

			//VERSION "SYMPA" (test grossier)
			D = (DistanceType)computeChi2Dist(statModel,&neighboursCloud,numberOfChi2Classes,includeNegValues,dumpHisto);
			//VERSION "SEVERE" (test ultra-precis)
			//D = (DistanceType)statModel->computeChi2Dist(theDistances,&Z,numberOfChi2Classes,dumpHisto);

			if (D>=0.0)
				D = sqrt(D);
		}

		//Version champ scalaire "IN" et "OUT" différents
		cell.points->setPointScalarValue(i,D);
	}

	return true;
}
