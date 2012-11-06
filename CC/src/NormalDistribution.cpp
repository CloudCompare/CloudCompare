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

#ifdef _MSC_VER
//To get rid of the really annoying warnings about unsafe methods
#pragma warning( disable: 4996 )
#endif

#include "NormalDistribution.h"
#include "GenericCloud.h"
#include "ErrorFunction.h"
#include "CCConst.h"
#include "DistanceComputationTools.h"
#include "ScalarFieldTools.h"

#include <math.h>
#include <assert.h>
#include <string.h>

using namespace CCLib;

NormalDistribution::NormalDistribution()
{
	parametersDefined = false;
}

NormalDistribution::NormalDistribution(DistanceType _mu, DistanceType _sigma2)
{
	setParameters(_mu,_sigma2);
}

bool NormalDistribution::getParameters(DistanceType &_mu, DistanceType &_sigma2) const
{
	_mu = mu;
	_sigma2 = sigma2;

	return parametersDefined;
}

bool NormalDistribution::setParameters(DistanceType _mu, DistanceType _sigma2)
{
	mu = _mu;
	sigma2 = _sigma2;

	//pour le test du Chi2
	chi2ClassesPositions.clear();
	Pi.clear();

	if (sigma2>0.0)
	{
		parametersDefined=true;
		qFactor=1.0f/(2.0f*sigma2);
		normFactor=1.0/sqrt(2.0*M_PI*(double)sigma2);
	}
	else
	{
		parametersDefined=false;
		qFactor=1.0;
		normFactor=1.0;
	}

	return parametersDefined;
}

double NormalDistribution::computeP(DistanceType x) const
{
	DistanceType p = (x-mu);
	return exp(-double(p*p*qFactor))*normFactor;
}

double NormalDistribution::computeP(DistanceType x1, DistanceType x2) const
{
	return 0.5*(ErrorFunction::erf(double(x2-mu)/sqrt(double(2.0*sigma2)))-ErrorFunction::erf(double(x1-mu)/sqrt(double(2.0*sigma2))));
}

double NormalDistribution::computePfromZero(DistanceType x) const
{
	return 0.5*(ErrorFunction::erf(double(x-mu)/sqrt(2.0*double(sigma2)))+1.0);
}

void NormalDistribution::getTextualDescription(char* buffer) const
{
    assert(buffer);

	if (parametersDefined)
		sprintf(buffer,"Normal [mean=%5.3f,sigma=%3.5f]",mu,sqrt(sigma2));
	else
		sprintf(buffer,"Undefined Normal Distribution");
}

bool NormalDistribution::computeParameters(const GenericCloud* Yk, bool includeNegValues)
{
	parametersDefined = false;

	int i,n = Yk->size();
	if (n==0)
		return false;

	double val, mean=0.0, stddev2=0.0;
	int counter=0;

	for (i=0;i<n;++i)
	{
		val = (double)Yk->getPointScalarValue(i);
		if (includeNegValues || val>=0.0)
		{
			mean += val;
			stddev2 += val*val;
			++counter;
		}
	}

	if (counter==0)
        return false;

    mean /= (double)counter;
    stddev2 = fabs(stddev2/(double)counter - mean*mean);
    return setParameters((DistanceType)mean,(DistanceType)stddev2);
}

bool NormalDistribution::computeParameters(const distancesContainer& values, bool includeNegValues)
{
	parametersDefined = false;

	unsigned i,n=values.size();
	if (n==0)
        return false;

	//compute mean and std. dev.
	double val, mean=0.0, stddev2=0.0;
	unsigned counter=0;

	for (i=0;i<n;++i)
	{
		val = (double)values[i];
		if (includeNegValues || val>=0.0)
		{
			mean += val;
			stddev2 += val*val;
			++counter;
		}
	}

	if (counter==0)
        return false;

    mean /= (double)counter;
    stddev2 = fabs(stddev2/(double)counter - mean*mean);
    return setParameters((DistanceType)mean,(DistanceType)stddev2);
}

bool NormalDistribution::computeRobustParameters(const distancesContainer& values, double nSigma, bool includeNegValues)
{
	if (!computeParameters(values,includeNegValues))
        return false;

	unsigned k,counter=0;
	double stddev = sqrt(sigma2)*nSigma;
	double mean=0.0,stddev2=0.0;
	DistanceType val;
	for (k=0;k<values.size();++k)
	{
	    val = values[k];
		if (fabs(val-mu)<stddev)
		{
			++counter;
			mean += (double)val;
			stddev2 += (double)val*val;
		}
	}

	if (counter==0)
        return false;

    mean /= (double)counter;
    stddev2 = fabs(stddev2/(double)counter - mean*mean);
    return setParameters((DistanceType)mean,(DistanceType)stddev2);
}

double NormalDistribution::computeChi2Dist(const GenericCloud* Yk, unsigned numberOfClasses, bool includeNegValues, int* histo)
{
	assert(Yk);

	unsigned i,n = Yk->size();

    //we must refine the real number of elements
    unsigned numberOfElements = n;
	if (!includeNegValues)
        numberOfElements = ScalarFieldTools::countScalarFieldPositiveValues(Yk);

    if (numberOfElements==0)
        return -1.0;

	if (numberOfClasses < 1 || numberOfClasses*numberOfClasses > numberOfElements)
		return -1.0;
	else if (numberOfClasses == 1)
		return 0.0;

	if (!setChi2ClassesPositions(numberOfClasses))
        return -1.0;

	assert(Pi.size() == numberOfClasses);

    int* _histo = histo;
	if (!_histo)
        _histo = new int[numberOfClasses];
	if (!_histo)
        return -1.0; //problème d'allocation

	memset(_histo,0,numberOfClasses*sizeof(int));

	//calcul de l'histogramme
	unsigned j;
	DistanceType V;
	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
		{
            for (j=0;j<numberOfClasses-1;++j)
                if (V<chi2ClassesPositions[j])
                    break;

            ++_histo[j];
		}
	}

	//calcul de la distance du Chi2
	DistanceType nPi,tempValue,dk = 0.0;
	for (i=0;i<numberOfClasses;++i)
	{
		nPi = Pi[i]*(DistanceType)numberOfElements;
		tempValue = (DistanceType)_histo[i]-nPi;
		dk += tempValue*tempValue/nPi;
	}

	if (_histo && !histo)
        delete[] _histo;
    _histo=0;

	return dk;
}

bool NormalDistribution::setChi2ClassesPositions(unsigned numberOfClasses)
{
	chi2ClassesPositions.clear();
	Pi.clear();

	if (!parametersDefined || numberOfClasses<2)
        return false;

	//cas très simple
	if (numberOfClasses==2)
	{
		Pi.push_back(0.5);
		chi2ClassesPositions.push_back(mu);
		Pi.push_back(0.5);
	}
	//cas général
	else //numberOfClasses>2
	{
		DistanceType x,y,oldy,sigma = sqrt(sigma2);
		//une première classe entre -inf et mu-2.sigma
		x = mu-2.0f*sigma;
		y = (DistanceType)computePfromZero(x);
		Pi.push_back(y);
		chi2ClassesPositions.push_back(x);

		//une serie de numberOfClasses-2 classes comprises entre mu-2.sigma et mu+2.sigma
		DistanceType pas = 4.0f*sigma/(DistanceType)(numberOfClasses-2);
		for (unsigned i=0;i<numberOfClasses-2;++i)
		{
			x = x+pas;
			oldy = y;
			y = (DistanceType)computePfromZero(x);
			Pi.push_back(y-oldy);
			chi2ClassesPositions.push_back(x);
		}

		//une dernière classe entre et mu+2.sigma et inf
		//x = mu+2.0*sigma;
		y = 1.0f-y;
		Pi.push_back(y);
	}

	return true;
}
